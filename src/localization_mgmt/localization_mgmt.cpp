/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "localization_mgmt.hpp"

namespace simulation_management_ros_tool {

LocalizationMgmt::LocalizationMgmt(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{private_node_handle},
          objectArray_{(params_.fromParamServer(), params_.tf_topic_map), params_.frame_id_objects_prefix},
          reconfigSrv_{private_node_handle}, tfListener_{tfBuffer_} {

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&LocalizationMgmt::reconfigureRequest, this, _1, _2));
    startTime = ros::Time::now();

    /**
     * Publishers & subscribers
     */
    objectsGroundTruthPub_ = node_handle.advertise<automated_driving_msgs::ObjectStateArray>(
        params_.objects_out_topic_with_ns, params_.msg_queue_size);

    objectInitializationSub_ = node_handle.subscribe(params_.object_initialization_in_topic_with_ns,
                                                     params_.msg_queue_size,
                                                     &LocalizationMgmt::objectInitializationSubCallback,
                                                     this,
                                                     ros::TransportHints().tcpNoDelay());
    objectRemovalSub_ = node_handle.subscribe(params_.object_removal_in_topic_with_ns,
                                              params_.msg_queue_size,
                                              &LocalizationMgmt::objectRemovalSubCallback,
                                              this,
                                              ros::TransportHints().tcpNoDelay());

    desiredMotionSub_ = node_handle.subscribe(params_.desired_motion_in_topic_with_ns,
                                              params_.msg_queue_size,
                                              &LocalizationMgmt::desiredMotionSubCallback,
                                              this,
                                              ros::TransportHints().tcpNoDelay());

    resetObjectPoseSub_ = node_handle.subscribe(params_.reset_object_pose_topic,
                                                params_.msg_queue_size,
                                                &LocalizationMgmt::resetObjectPoseCallback,
                                                this,
                                                ros::TransportHints().tcpNoDelay());

    timer_ = private_node_handle.createTimer(
        ros::Duration(1.0 / params_.loc_mgmt_freq), &LocalizationMgmt::objectStatePublisher, this);
}

/**
 * @brief assigns the incoming desired motion to the respective object (if exists)
 * @param msg
 */
void LocalizationMgmt::desiredMotionSubCallback(const simulation_only_msgs::DeltaTrajectoryWithID& msg) {

    if (objectArray_.checkObjectExistence(msg.object_id) == true) {

        objectArray_.getObjectStateById(msg.object_id)->newDeltaTrajectory(msg, ros::Time::now());

    } else {
        ROS_WARN("%s: Received DesiredMotion.msg of Object that does not exist! I "
                 "discard this message! (id=%s)",
                 ros::this_node::getName().c_str(),
                 std::to_string(msg.object_id).c_str());
    }
}

/**
 * @brief Callback for object initializations
 * @param msg
 */
void LocalizationMgmt::objectInitializationSubCallback(const simulation_only_msgs::ObjectInitialization& msg) {

    if (objectArray_.checkObjectExistence(msg.object_id) == false) {

        objectArray_.initializeObject(msg, ros::Time::now());
        ROS_INFO("%s: Initialized object with id %s",
                 ros::this_node::getName().c_str(),
                 std::to_string(msg.object_id).c_str());

    } else {
        ROS_WARN("%s: Received ObjectInitialization.msg of Object that already was initialized! I "
                 "discard this message! (id=%s)",
                 ros::this_node::getName().c_str(),
                 std::to_string(msg.object_id).c_str());
    }
}

void LocalizationMgmt::objectRemovalSubCallback(const simulation_only_msgs::ObjectRemoval& msg) {

    if (objectArray_.checkObjectExistence(msg.object_id)) {

        objectArray_.removeObject(msg.object_id);
        ROS_INFO(
            "%s: Removed object with id %s", ros::this_node::getName().c_str(), std::to_string(msg.object_id).c_str());

    } else {
        ROS_WARN("%s: Received ObjectRemoval.msg for Object that does not exist! I "
                 "discard this message! (id=%s)",
                 ros::this_node::getName().c_str(),
                 std::to_string(msg.object_id).c_str());
    }
}

/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void LocalizationMgmt::reconfigureRequest(LocalizationMgmtConfig& config, uint32_t level) {
    params_.fromConfig(config);
    timer_.setPeriod(ros::Duration(1.0 / params_.loc_mgmt_freq));
}

/**
 * @brief Interpolates all object states and publishes them
 * @param Called on a timer event which is not further used
 */
void LocalizationMgmt::objectStatePublisher(const ros::TimerEvent& event) {

    ros::Time timestamp = ros::Time::now();

    if (objectArray_.containsObjects()) {

//        objectArray_.determineActiveStateAndInterpolatePoses(timestamp);
        objectsGroundTruthPub_.publish(objectArray_.activeObjectsToMsg(timestamp));
        broadcastTF();

    } else {
        if ((timestamp - startTime).toSec() > params_.delay_for_no_objects_warning)
            ROS_WARN_THROTTLE(1, "%s: Publishing but no objects in memory!", ros::this_node::getName().c_str());
    }
}

/**
 * @brief Broadcasts the current object poses as tf
 */
void LocalizationMgmt::broadcastTF() {

    std::vector<localization_mgmt_types::dyn_obj_ptr_t> objectStateVector = objectArray_.getActiveObjectStates();
    for (localization_mgmt_types::dyn_obj_ptr_t objPtr : objectStateVector) {
        bool transformValid;
        geometry_msgs::TransformStamped transform;
        objPtr->getTransformStamped(transform, transformValid);
        if (transformValid) {
            tfBroadcaster_.sendTransform(transform);
        }
}

void LocalizationMgmt::resetObjectPoseCallback(const automated_driving_msgs::ObjectState& msg){
    if(objectArray_.checkObjectExistence(msg.object_id)) {
        localization_mgmt_types::dyn_obj_ptr_t objStatePtr = objectArray_.getObjectStateById(msg.object_id);

        objStatePtr->setCurrPose(msg.motion_state.pose.pose);



    }else {
        ROS_WARN("%s: Received ObjectState.msg of Object that does not exist! I "
                         "discard this message! (id=%s)",
                 ros::this_node::getName().c_str(),
                 std::to_string(msg.object_id).c_str());
    }
}

} // namespace simulation_management_ros_tool
