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

#pragma once

#include <time.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <automated_driving_msgs/MotionPrediction.h>
#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <simulation_only_msgs/DeltaTrajectoryWithID.h>
#include <simulation_only_msgs/ObjectInitialization.h>
#include <simulation_only_msgs/ObjectRemoval.h>

#include "localization_mgmt_types.hpp"
#include "simulation_management_ros_tool/LocalizationMgmtInterface.h"


namespace simulation_management_ros_tool {

class LocalizationMgmt {
public:
    LocalizationMgmt(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher objectsGroundTruthPub_;
    ros::Subscriber desiredMotionSub_;
    ros::Subscriber objectInitializationSub_;
    ros::Subscriber objectRemovalSub_;
    ros::Subscriber resetObjectPoseSub_;

    LocalizationMgmtInterface params_;

    // storage for the object states
    localization_mgmt_types::DynamicObjectArray objectArray_;

    // ros timer for publishing with constant frequency
    ros::Timer timer_;

    ros::Time startTime;

    dynamic_reconfigure::Server<LocalizationMgmtConfig> reconfigSrv_; // Dynamic reconfiguration service

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    void broadcastTF();
    void objectStatePublisher(const ros::TimerEvent& event);
    void desiredMotionSubCallback(const simulation_only_msgs::DeltaTrajectoryWithID& msg);
    void objectInitializationSubCallback(const simulation_only_msgs::ObjectInitialization& msg);
    void objectRemovalSubCallback(const simulation_only_msgs::ObjectRemoval& msg);
    void reconfigureRequest(LocalizationMgmtConfig&, uint32_t);
    void resetObjectPoseCallback(const automated_driving_msgs::ObjectState& msg);
};

} // namespace simulation_management_ros_tool
