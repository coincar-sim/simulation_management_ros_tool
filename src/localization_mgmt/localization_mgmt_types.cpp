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

#include <util_geometry_msgs/util_geometry_msgs.hpp>
#include <util_simulation_only_msgs/util_simulation_only_msgs.hpp>

#include "localization_mgmt_types.hpp"

namespace localization_mgmt_types {

DynamicObject::DynamicObject(const simulation_only_msgs::ObjectInitialization& initMsg,
                             const ros::Time& initTimestamp,
                             const std::string& frameId,
                             const std::string& frameIdObjectsPrefix) {
    objectID_ = initMsg.object_id;
    objectClassification_ = initMsg.classification;
    poseAtStartOfDeltaTraj_ = initMsg.initial_pose;

    timestampSpawn_ = initTimestamp + initMsg.spawn_time;

    if (initMsg.spawn_time.toNSec() > 0) {
        objectActive_ = false;
    } else {
        objectActive_ = true;
    }

//    startTimeOfDeltaTrajNsec_ = timestampSpawn_.toNSec();
    deltaTrajectoryWithID_ = initMsg.initial_delta_trajectory;
    deltaTrajectoryWithID_.header.stamp = timestampSpawn_;

    frameId_ = frameId;
    childFrameId_ = frameIdObjectsPrefix + std::to_string(objectID_).c_str();
    hull_ = initMsg.hull;

    switch (initMsg.role.type) {
    case simulation_only_msgs::ObjectRole::OBSTACLE_STATIC:
        objectRole_ = OBJECT_ROLE::OBSTACLE_STATIC;
        break;
    case simulation_only_msgs::ObjectRole::OBSTACLE_DYNAMIC:
        objectRole_ = OBJECT_ROLE::OBSTACLE_DYNAMIC;
        break;
    case simulation_only_msgs::ObjectRole::AGENT_OPERATED:
        objectRole_ = OBJECT_ROLE::AGENT_OPERATED;
        break;
    default:
        throw std::runtime_error("ObjectRole \"" + std::to_string(initMsg.role.type) + "\" not in known");
    }
    if (objectRole_ == OBJECT_ROLE::OBSTACLE_DYNAMIC) {
        timestampRemoval_ = initTimestamp + initMsg.spawn_time +
                            initMsg.initial_delta_trajectory.delta_poses_with_delta_time.back().delta_time;
    }
    if (objectRole_ == OBJECT_ROLE::OBSTACLE_STATIC) {
        currPose_ = poseAtStartOfDeltaTraj_;
    }
}


void DynamicObject::newDeltaTrajectory(const simulation_only_msgs::DeltaTrajectoryWithID& deltaTrajectory,
                                       const ros::Time& timestamp) {
    if (util_simulation_only_msgs::containsNANs(deltaTrajectory)) {
        ROS_WARN_THROTTLE(1,
                          "Not regarding desired motion of object with id %s as it contains NANs",
                          std::to_string(objectID_).c_str());
        return;
    }

    if (objectRole_ != OBJECT_ROLE::AGENT_OPERATED) {
        ROS_WARN_THROTTLE(1,
                          "Not regarding desired motion of object with id %s as this is not an operated agent",
                          std::to_string(objectID_).c_str());
        return;
    }

    interpolatePose(timestamp);
    poseAtStartOfDeltaTraj_ = currPose_;
    deltaTrajectoryWithID_ = deltaTrajectory;
    deltaTrajectoryWithID_.header.stamp = timestamp;
//    startTimeOfDeltaTrajNsec_ = timestamp.toNSec();
}

void DynamicObject::interpolatePose(const ros::Time& timestamp) {
    try {

        if (timestamp == timestampOfLastUpdate_) {
            // if timestamp is identical, do not recalculate
            return;
        }


        if (timestamp < timestampSpawn_) {
            // if object is not yet spawned, it is inactive
            objectActive_ = false;
            timestampOfLastUpdate_ = timestamp;
            return;
        }

        if (objectRole_ == OBJECT_ROLE::OBSTACLE_STATIC) {
            // for a static obstacle the position does not have to be recalculated
            objectActive_ = true;
            timestampOfLastUpdate_ = timestamp;
            return;
        }

        if (objectRole_ == OBJECT_ROLE::OBSTACLE_DYNAMIC) {
            if (timestamp > timestampRemoval_) {
                // if a dynamic obstacle has reached the end of its trajectory, it is inactive
                objectActive_ = false;
                timestampOfLastUpdate_ = timestamp;
                return;
            }
        }


        geometry_msgs::Pose newDeltaPose;
        bool valid;
        std::string errMsg;
        util_simulation_only_msgs::interpolateDeltaPose(deltaTrajectoryWithID_, timestamp, newDeltaPose, valid, errMsg);

        if (valid) {

            objectActive_ = true;
            geometry_msgs::Pose newPose = util_geometry_msgs::computations::addDeltaPose(poseAtStartOfDeltaTraj_, newDeltaPose);
            if (util_geometry_msgs::checks::containsNANs(newPose)) {
                ROS_ERROR_THROTTLE(
                    1, "Not updating motion state of object with id %s as computed pose contains NANs!", std::to_string(objectID_).c_str());
                return;
            }
            currPose_ = newPose;


        } else {
            ROS_WARN_THROTTLE(1,
                              "Not updating motion state of object with id %s as interpolation not valid: \"%s\")",
                              std::to_string(objectID_).c_str(),
                              errMsg.c_str());
        }
        timestampOfLastUpdate_ = timestamp;

    } catch (std::exception& e) {
        ROS_WARN_THROTTLE(1,
                          "Not updating motion state of object with id %s as following error "
                          "occured: \"%s\")",
                          std::to_string(objectID_).c_str(),
                          e.what());
        timestampOfLastUpdate_ = timestamp;
    }
}

bool DynamicObject::isActive() {
    return objectActive_;
}


automated_driving_msgs::ObjectState DynamicObject::toMsg(const ros::Time& timestamp) {

    if (timestamp != timestampOfLastUpdate_) {
        throw std::runtime_error("Requested timestamp of message differs from timestamp of objectState");
    }
    automated_driving_msgs::ObjectState os;
    os.header.stamp = timestampOfLastUpdate_;
    os.header.frame_id = frameId_;
    os.object_id = objectID_;
    os.classification = objectClassification_;
    os.existence_probability = 1.0;
    os.motion_state = newMotionStatePoseOnly();
    assert(!util_geometry_msgs::checks::containsNANs(currPose_));
    os.motion_state.pose.pose = currPose_;
    os.motion_state.header.stamp = timestamp;
    os.motion_state.header.frame_id = frameId_;
    os.motion_state.child_frame_id = childFrameId_;
    os.hull = hull_;
    return os;
    // check if contains NANs
}

void DynamicObject::getTransformStamped(geometry_msgs::TransformStamped& tfs, bool& valid) {

    assert(!util_geometry_msgs::checks::containsNANs(currPose_));

    tfs.header.stamp = timestampOfLastUpdate_;
    tfs.header.frame_id = frameId_;
    // add prefix to prevent usage of TF instead of motion state; TF is only provided for visualization
    tfs.child_frame_id = "visualization_only__" + childFrameId_;
    tfs.transform = util_geometry_msgs::conversions::transformFromPose(currPose_);
    if (!util_geometry_msgs::checks::quaternionNormalized(tfs.transform.rotation)) {
        ROS_WARN_THROTTLE(1,
                          "No valid transform for object with id %s as quaternion not normalized",
                          std::to_string(objectID_).c_str());
        valid = false;
        return;
    }

    valid = true;
}

void DynamicObject::setCurrPose(geometry_msgs::Pose pose) {
    currPose_ = pose;
    poseAtStartOfDeltaTraj_ = pose;
}


DynamicObjectArray::DynamicObjectArray(std::string frameId, std::string frameIdObjectsPrefix) {
    frameId_ = frameId;
    frameIdObjectsPrefix_ = frameIdObjectsPrefix;
}

void DynamicObjectArray::setFrameIds(std::string frameId, std::string frameIdObjectsPrefix) {
    frameId_ = frameId;
    frameIdObjectsPrefix_ = frameIdObjectsPrefix;
}

void DynamicObjectArray::initializeObject(const simulation_only_msgs::ObjectInitialization& msg,
                                          const ros::Time& timestamp) {

    if (checkObjectExistence(msg.object_id)) {
        throw std::runtime_error("Object with id " + std::to_string(msg.object_id) + " already initialized");
    }

    if (!(msg.header.frame_id == frameId_)) {
        ROS_ERROR_STREAM("Object with id " + std::to_string(msg.object_id) +
                         " not initialized: its initial position is in frame " + msg.header.frame_id +
                         " but should be in " + frameId_);
    }

    dyn_obj_ptr_t& currentObjectPtr = objectStateMap_[msg.object_id];
    currentObjectPtr = std::make_shared<DynamicObject>(msg, timestamp, frameId_, frameIdObjectsPrefix_);
}

void DynamicObjectArray::determineActiveStateAndInterpolatePoses(const ros::Time& timestamp) {

    for (auto it : objectStateMap_) {
        dyn_obj_ptr_t objPtr = it.second;
        objPtr->interpolatePose(timestamp);
    }
    timestampOfLastUpdate_ = timestamp;
}

void DynamicObjectArray::removeObject(const unsigned int objectId) {
    objectStateMap_.erase(objectId);
}

bool DynamicObjectArray::containsObjects() {
    return !objectStateMap_.empty();
}

bool DynamicObjectArray::checkObjectExistence(const unsigned int objectId) {
    return (objectStateMap_.count(objectId) > 0);
}

dyn_obj_ptr_t DynamicObjectArray::getObjectStateById(const unsigned int objectId) {
    auto it = objectStateMap_.find(objectId);
    if (it == objectStateMap_.end()) {
        throw std::runtime_error("Object with id " + std::to_string(objectId) + "not in List");
    }
    return it->second;
}

std::vector<dyn_obj_ptr_t> DynamicObjectArray::getAllObjectStates() {

    std::vector<dyn_obj_ptr_t> objectStateVector;
    std::transform(objectStateMap_.begin(),
                   objectStateMap_.end(),
                   std::back_inserter(objectStateVector),
                   [](auto& mapPair) { return mapPair.second; });
    return objectStateVector;
}

std::vector<dyn_obj_ptr_t> DynamicObjectArray::getActiveObjectStates() {

    std::vector<dyn_obj_ptr_t> objectStateVector;
    // todo: better implementation
    for (auto it : objectStateMap_) {
        dyn_obj_ptr_t objPtr = it.second;
        if (objPtr->isActive()) {
            objectStateVector.push_back(objPtr);
        }
    }
    return objectStateVector;
}

automated_driving_msgs::ObjectStateArray DynamicObjectArray::activeObjectsToMsg(const ros::Time& timestamp) {

    determineActiveStateAndInterpolatePoses(timestamp);
    automated_driving_msgs::ObjectStateArray osa;
    osa.header.stamp = timestamp;
    osa.header.frame_id = frameId_;
    std::vector<dyn_obj_ptr_t> objectStateVector = getActiveObjectStates();
    osa.objects = std::vector<automated_driving_msgs::ObjectState>(objectStateVector.size());
    for (size_t i = 0; i < objectStateVector.size(); i++) {
        osa.objects[i] = objectStateVector[i]->toMsg(timestamp);
    }
    return osa;
}

automated_driving_msgs::MotionState newMotionStatePoseOnly() {

    automated_driving_msgs::MotionState ms;
    ms.pose.covariance = util_geometry_msgs::checks::covarianceGroundTruthValues;
    ms.twist.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    ms.accel.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    return ms;
}

} // namespace localization_mgmt_types
