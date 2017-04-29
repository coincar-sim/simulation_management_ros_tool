#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Header.h>
#include "simulation_management_ros_tool/TimeMgmtParameters.h"

namespace simulation_management_ros_tool {

class TimeMgmt {
public:
	TimeMgmt(ros::NodeHandle, ros::NodeHandle);

private:
	void timerCallbackSim(const ros::WallTimerEvent&);
	void reconfigureRequest(TimeMgmtConfig&, uint32_t);

	dynamic_reconfigure::Server<TimeMgmtConfig> reconfigSrv_; // Dynamic reconfiguration service

	TimeMgmtParameters params_;

	ros::Publisher clockPub_;
	ros::WallTimer timer_;
	rosgraph_msgs::Clock msgClock_;
	ros::WallTime startWallTime_;
	ros::Time startSimTime_;
	ros::Time currentSimTime_;

};

} // namespace simulation_management_ros_tool
