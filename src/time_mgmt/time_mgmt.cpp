#include "time_mgmt.hpp"

namespace simulation_management_ros_tool {

TimeMgmt::TimeMgmt(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&TimeMgmt::reconfigureRequest, this, _1, _2));

    /**
     * Set up publisher & timer
     */
    clockPub_ = private_node_handle.advertise<rosgraph_msgs::Clock>("/clock", 5);
    timer_ = private_node_handle.createWallTimer(
        ros::WallDuration(1.0 / params_.time_resolution / params_.acc_factor), &TimeMgmt::timerCallbackSim, this);
    startWallTime_ = ros::WallTime::now();
    startSimTime_ = startSimTime_.fromNSec(startWallTime_.toNSec());
}

/**
 * This callback is called by the timer and publishes on the clock topic
 */
void TimeMgmt::timerCallbackSim(const ros::WallTimerEvent&) {
    ros::WallDuration currentRealTimeDifference = ros::WallTime::now() - startWallTime_;
    ros::Duration currentSimTimeDifference;
    currentSimTimeDifference =
        currentSimTimeDifference.fromNSec(currentRealTimeDifference.toNSec() * params_.acc_factor);
    currentSimTime_ = startSimTime_ + currentSimTimeDifference;

    rosgraph_msgs::Clock msgClock;
    msgClock.clock = currentSimTime_;
    clockPub_.publish(msgClock);
}

/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void TimeMgmt::reconfigureRequest(TimeMgmtConfig& config, uint32_t level) {
    // update parameters
    params_.fromConfig(config);

    // renew start_time as the parameter acc_factor could have changed
    startWallTime_ = ros::WallTime::now();
    startSimTime_ = currentSimTime_;

    // renew Wall Timer parameters as time_resolution or acc_factor could have changed
    timer_.setPeriod(ros::WallDuration(1.0 / params_.time_resolution / params_.acc_factor));
}

} // namespace simulation_management_ros_tool
