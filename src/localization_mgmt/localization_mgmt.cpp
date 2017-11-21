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

    desiredMotionSub_ = node_handle.subscribe(params_.desired_motion_in_topic_with_ns,
                                              params_.msg_queue_size,
                                              &LocalizationMgmt::desiredMotionSubCallback,
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
        ROS_WARN("%s: Received DesiredMotion.msg of Object before its initialization! I "
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

    } else {
        ROS_WARN("%s: Received ObjectInitialization.msg of Object that already was initialized! I "
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

    if (objectArray_.getAllObjectStates().size() != 0) {

        objectArray_.interpolatePoses(timestamp);
        objectsGroundTruthPub_.publish(objectArray_.toMsg(timestamp));
        broadcastTF();

    } else {
        ROS_WARN_THROTTLE(1, "%s: Publishing but no objects in memory!", ros::this_node::getName().c_str());
    }
}

/**
 * @brief Broadcasts the current object poses as tf
 */
void LocalizationMgmt::broadcastTF() {

    std::vector<localization_mgmt_types::dyn_obj_ptr_t> objectStateVector = objectArray_.getAllObjectStates();
    for (localization_mgmt_types::dyn_obj_ptr_t objPtr : objectStateVector) {

        tfBroadcaster_.sendTransform(objPtr->toTransformStamped());
    }
}

} // namespace simulation_management_ros_tool
