#include "localization_mgmt_types.hpp"

namespace localization_mgmt_types {

DynamicObject::DynamicObject(const simulation_only_msgs::ObjectInitialization& initMsg,
                             const ros::Time& initTimestamp,
                             const std::string& frameId,
                             const std::string& frameIdObjectsPrefix) {
    objectID_ = initMsg.object_id;
    objectClassification_ = initMsg.classification;
    poseAtStartOfDeltaTraj_ = initMsg.initial_pose;
    startTimeOfDeltaTrajNsec_ = initTimestamp.toNSec();
    deltaTrajectoryWithID_ = initMsg.initial_delta_trajectory;
    frameId_ = frameId;
    childFrameId_ = frameIdObjectsPrefix + std::to_string(objectID_).c_str();
    hull_ = initMsg.hull;
}


void DynamicObject::newDeltaTrajectory(const simulation_only_msgs::DeltaTrajectoryWithID& deltaTrajectory,
                                       const ros::Time& timestamp) {
    if (util_localization_mgmt::deltaTrajectoryContainsNANs(deltaTrajectory)) {
        ROS_WARN_THROTTLE(1,
                          "Not regarding desired motion of object with id %s as it contains NANs",
                          std::to_string(objectID_).c_str());
        return;
    }
    interpolatePose(timestamp);
    poseAtStartOfDeltaTraj_ = currPose_;
    deltaTrajectoryWithID_ = deltaTrajectory;
    startTimeOfDeltaTrajNsec_ = timestamp.toNSec();
    // check if delta pose contains NANs
}

void DynamicObject::interpolatePose(const ros::Time& timestamp) {
    try {

        if (timestamp.toNSec() == currTimeNsec_) {
            return;
        }

        double scale;
        size_t i;

        std::tie(i, scale) = util_localization_mgmt::getInterpolationIndexAndScale(
            deltaTrajectoryWithID_, startTimeOfDeltaTrajNsec_, timestamp);

        geometry_msgs::Pose p0 = deltaTrajectoryWithID_.delta_poses_with_delta_time[i].delta_pose;

        geometry_msgs::Pose p1 = deltaTrajectoryWithID_.delta_poses_with_delta_time[i + 1].delta_pose;
        geometry_msgs::Pose newDeltaPose = util_localization_mgmt::interpolatePose(p0, p1, scale);

        currTimeNsec_ = timestamp.toNSec();
        currPose_ = util_localization_mgmt::addDeltaPose(poseAtStartOfDeltaTraj_, newDeltaPose);

    } catch (std::exception& e) {
        ROS_WARN_THROTTLE(1,
                          "Not updating motion state of object with id %s as following error "
                          "occured: \"%s\")",
                          std::to_string(objectID_).c_str(),
                          e.what());
        currTimeNsec_ = timestamp.toNSec();
    }
}


automated_driving_msgs::ObjectState DynamicObject::toMsg() {
    automated_driving_msgs::ObjectState os;
    ros::Time timestamp = util_localization_mgmt::rosTimeFromNsec(currTimeNsec_);
    os.header.stamp = timestamp;
    os.header.frame_id = frameId_;
    os.object_id = objectID_;
    os.classification = objectClassification_;
    os.existence_probability = 1.0;
    os.motion_state = util_localization_mgmt::newMotionStatePoseOnly();
    os.motion_state.pose.pose = currPose_;
    os.motion_state.header.stamp = timestamp;
    os.motion_state.header.frame_id = frameId_;
    os.motion_state.child_frame_id = childFrameId_;
    os.hull = hull_;
    return os;
    // check if contains NANs
}

geometry_msgs::TransformStamped DynamicObject::toTransformStamped() {
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = util_localization_mgmt::rosTimeFromNsec(currTimeNsec_);
    tfs.header.frame_id = frameId_;
    tfs.child_frame_id = "visualization_only__" + childFrameId_;
    tfs.transform = util_localization_mgmt::transformFromPose(currPose_);
    return tfs;
    // check if contains NANs
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

void DynamicObjectArray::interpolatePoses(const ros::Time& timestamp) {
    std::vector<dyn_obj_ptr_t> objectStateVector = getAllObjectStates();
    for (dyn_obj_ptr_t objPtr : objectStateVector) {
        objPtr->interpolatePose(timestamp);
    }
}

bool DynamicObjectArray::checkObjectExistence(const int objectId) {
    return (objectStateMap_.count(objectId) > 0);
}

dyn_obj_ptr_t DynamicObjectArray::getObjectStateById(const int objectId) {
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

automated_driving_msgs::ObjectStateArray DynamicObjectArray::toMsg(const ros::Time& timestamp) {

    automated_driving_msgs::ObjectStateArray osa;
    osa.header.stamp = timestamp;
    osa.header.frame_id = frameId_;
    std::vector<dyn_obj_ptr_t> objectStateVector = getAllObjectStates();
    osa.objects = std::vector<automated_driving_msgs::ObjectState>(objectStateVector.size());
    for (size_t i = 0; i < objectStateVector.size(); i++) {
        osa.objects[i] = objectStateVector[i]->toMsg();
    }
    return osa;
}

} // namespace localization_mgmt_types
