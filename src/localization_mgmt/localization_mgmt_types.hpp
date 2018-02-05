#pragma once

#include <unordered_map>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>

#include <simulation_utils/util_localization_mgmt.hpp>
#include "automated_driving_msgs/MotionPrediction.h"
#include "automated_driving_msgs/MotionState.h"
#include "automated_driving_msgs/ObjectClassification.h"
#include "automated_driving_msgs/ObjectState.h"
#include "automated_driving_msgs/ObjectStateArray.h"
#include "simulation_only_msgs/DeltaTrajectoryWithID.h"
#include "simulation_only_msgs/ObjectInitialization.h"
#include "simulation_only_msgs/ObjectRemoval.h"
#include "simulation_only_msgs/ObjectRole.h"


namespace localization_mgmt_types {


enum class OBJECT_ROLE { OBSTACLE_STATIC = 10, OBSTACLE_DYNAMIC = 20, AGENT_OPERATED = 100 };


class DynamicObject {
public:
    DynamicObject(const simulation_only_msgs::ObjectInitialization& initMsg,
                  const ros::Time& initTimestamp,
                  const std::string& frameId,
                  const std::string& frameIdObjectsPrefix);

    void newDeltaTrajectory(const simulation_only_msgs::DeltaTrajectoryWithID&, const ros::Time& timestamp);
    void interpolatePose(const ros::Time& timestamp);
    bool isActive();

    automated_driving_msgs::ObjectState toMsg(const ros::Time& timestamp);
    geometry_msgs::TransformStamped toTransformStamped();

private:
    int objectID_;

    uint64_t startTimeOfDeltaTrajNsec_;
    uint64_t spawnTimeNSec_;
    ros::Time timestampOfLastUpdate_;
    ros::Time timestampSpawn_;
    ros::Time timestampRemoval_;
    geometry_msgs::Pose currPose_;
    geometry_msgs::Pose poseAtStartOfDeltaTraj_;

    std::string frameId_;
    std::string childFrameId_;

    OBJECT_ROLE objectRole_;
    bool objectActive_;

    automated_driving_msgs::ObjectClassification objectClassification_;
    simulation_only_msgs::DeltaTrajectoryWithID deltaTrajectoryWithID_;
    shape_msgs::Mesh hull_;
};

typedef std::shared_ptr<DynamicObject> dyn_obj_ptr_t;

class DynamicObjectArray {
public:
    DynamicObjectArray(std::string frameId = "frameId", std::string frameIdObjectsPrefix = "frameIdObjectsPrefix");

    void setFrameIds(std::string frameId, std::string frameIdObjectsPrefix);
    void initializeObject(const simulation_only_msgs::ObjectInitialization& msg, const ros::Time& timestamp);
    void determineActiveStateAndInterpolatePoses(const ros::Time& timestamp);
    void removeObject(const int objectId);
    bool containsObjects();
    bool checkObjectExistence(const int objectId);
    dyn_obj_ptr_t getObjectStateById(const int objectId);
    std::vector<dyn_obj_ptr_t> getActiveObjectStates();

    automated_driving_msgs::ObjectStateArray activeObjectsToMsg(const ros::Time& timestamp);

private:
    std::string frameId_;
    std::string frameIdObjectsPrefix_;
    ros::Time timestampOfLastUpdate_;
    std::unordered_map<int, dyn_obj_ptr_t> objectStateMap_;

    std::vector<dyn_obj_ptr_t> getAllObjectStates();
};

} // namespace localization_mgmt_types
