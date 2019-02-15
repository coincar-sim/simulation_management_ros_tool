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
    timer_ = private_node_handle.createWallTimer(ros::WallDuration(1.0 / params_.time_resolution / params_.acc_factor),
                                                 &TimeMgmt::timerCallbackSim,
                                                 this,
                                                 false,
                                                 !params_.pause_time);
    startWallTime_ = ros::WallTime::now();
    startSimTime_ = startSimTime_.fromNSec(startWallTime_.toNSec());

    // If simulation starts paused, publish /clock once for ros::Time::now() to be valid
    if (params_.pause_time) {
        rosgraph_msgs::Clock msgClock;
        msgClock.clock = startSimTime_;
        clockPub_.publish(msgClock);
    }
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

    // pause the timer when requested via the "pause" button
    if (params_.pause_time) {
        timer_.stop();
    } else {
        timer_.start();
    }
}

} // namespace simulation_management_ros_tool
