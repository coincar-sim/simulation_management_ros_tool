#pragma once

#include <time.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "localization_mgmt_types.hpp"
#include "localization_mgmt_util.hpp"
#include "automated_driving_msgs/MotionPrediction.h"
#include "automated_driving_msgs/MotionState.h"
#include "automated_driving_msgs/ObjectStateArray.h"
#include "simulation_only_msgs/DeltaTrajectoryWithID.h"
#include "simulation_only_msgs/ObjectInitialization.h"
#include "simulation_management_ros_tool/LocalizationMgmtParameters.h"

namespace simulation_management_ros_tool {

class LocalizationMgmt {
public:
	LocalizationMgmt(ros::NodeHandle, ros::NodeHandle);

private:
	ros::Publisher objectsGroundTruthPub_;
	ros::Subscriber desiredMotionSub_;
	ros::Subscriber objectInitializationSub_;

	LocalizationMgmtParameters params_;

	// storage for the object states
	localization_mgmt_types::DynamicObjectArray objectArray_;

	// ros timer for publishing with constant frequency
	ros::Timer timer_;

	dynamic_reconfigure::Server<LocalizationMgmtConfig> reconfigSrv_; // Dynamic reconfiguration service

	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
	tf2_ros::TransformBroadcaster tfBroadcaster_;

	void broadcastTF();
	void objectStatePublisher(const ros::TimerEvent& event);
	void desiredMotionSubCallback(
			const simulation_only_msgs::DeltaTrajectoryWithID& msg);
	void objectInitializationSubCallback(
			const simulation_only_msgs::ObjectInitialization& msg);
	void reconfigureRequest(LocalizationMgmtConfig&, uint32_t);

};

} // namespace simulation_management_ros_tool