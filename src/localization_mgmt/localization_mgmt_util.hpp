#pragma once

#include <stdexcept>
#include <cmath>
#include <tuple>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "automated_driving_msgs/MotionState.h"
#include "simulation_only_msgs/DeltaTrajectoryWithID.h"

namespace localization_mgmt_util {

geometry_msgs::Point interpolatePosition(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1, const double scale);

geometry_msgs::Quaternion interpolateOrientation(
		const geometry_msgs::Quaternion& o0,
		const geometry_msgs::Quaternion& o1, const double scale);

geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& p0,
		const geometry_msgs::Pose& p1, const double scale);

geometry_msgs::Pose calculatePose(const geometry_msgs::Pose& startPose,
		const geometry_msgs::Pose& deltaPose);

std::tuple<size_t, double> getInterpolationIndexAndScale(
		const simulation_only_msgs::DeltaTrajectoryWithID& dtwid,
		const uint64_t startTimeDeltaTrajectory,
		const ros::Time& interpolationTimestamp);

geometry_msgs::Transform transformFromPose(const geometry_msgs::Pose& pose);

automated_driving_msgs::MotionState newMotionStatePoseOnly();

boost::array<double, 36> covGroundTruth();

boost::array<double, 36> covUnknown();

ros::Time rosTimeFromNsec(const uint64_t nsec);

void checkScaleForInterpolation(const double scale);

bool deltaTrajectoryContainsNANs(const simulation_only_msgs::DeltaTrajectoryWithID& dtwid);

} // namespace localization_mgmt_util
