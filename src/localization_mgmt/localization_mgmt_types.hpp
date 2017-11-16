#pragma once

#include <unordered_map>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>

#include "automated_driving_msgs/MotionPrediction.h"
#include "automated_driving_msgs/MotionState.h"
#include "automated_driving_msgs/ObjectClassification.h"
#include "automated_driving_msgs/ObjectState.h"
#include "automated_driving_msgs/ObjectStateArray.h"
#include "simulation_only_msgs/DeltaTrajectoryWithID.h"
#include "simulation_only_msgs/ObjectInitialization.h"
#include <simulation_utils/util_localization_mgmt.hpp>


namespace localization_mgmt_types {

class DynamicObject {
public:
	DynamicObject(const simulation_only_msgs::ObjectInitialization& initMsg,
			const ros::Time& initTimestamp, const std::string& frameId,
			const std::string& objectsPrefixTf,
			const automated_driving_msgs::ObjectClassification& classification =
					automated_driving_msgs::ObjectClassification());

	void newDeltaTrajectory(const simulation_only_msgs::DeltaTrajectoryWithID&,
			const ros::Time& timestamp);
	void interpolatePose(const ros::Time& timestamp);

	automated_driving_msgs::ObjectState toMsg();
	geometry_msgs::TransformStamped toTransformStamped();

private:
	int objectID_;

	uint64_t startTimeOfDeltaTrajNsec_;
	uint64_t currTimeNsec_;
	geometry_msgs::Pose currPose_;
	geometry_msgs::Pose poseAtStartOfDeltaTraj_;

	std::string frameId_;
	std::string childFrameId_;

	automated_driving_msgs::ObjectClassification objectClassification_;
	simulation_only_msgs::DeltaTrajectoryWithID deltaTrajectoryWithID_;
	shape_msgs::Mesh hull_;
};

typedef std::shared_ptr<DynamicObject> dyn_obj_ptr_t;

class DynamicObjectArray {
public:
	DynamicObjectArray(std::string frameId = "frameId",
			std::string objectsPrefixTf = "objectsPrefixTf");

	void setFrameIds(std::string frameId, std::string objectsPrefixTf);
	void initializeObject(const simulation_only_msgs::ObjectInitialization& msg,
			const ros::Time& timestamp);
	void interpolatePoses(const ros::Time& timestamp);
	bool checkObjectExistence(const int objectId);
	dyn_obj_ptr_t getObjectStateById(const int objectId);
	std::vector<dyn_obj_ptr_t> getAllObjectStates();

	automated_driving_msgs::ObjectStateArray toMsg(const ros::Time& timestamp);

private:
	std::string frameId_;
	std::string objectsPrefixTf_;
	std::unordered_map<int, dyn_obj_ptr_t> objectStateMap_;
};

} // namespace localization_mgmt_types
