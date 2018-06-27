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

#include <unordered_map>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>

#include <automated_driving_msgs/MotionPrediction.h>
#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <simulation_only_msgs/DeltaTrajectoryWithID.h>
#include <simulation_only_msgs/ObjectInitialization.h>
#include <simulation_only_msgs/ObjectRemoval.h>


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
    void getTransformStamped(geometry_msgs::TransformStamped& tfs, bool& valid);

    void setCurrPose(geometry_msgs::Pose pose);
private:
    unsigned int objectID_;

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

    void initializeObject(const simulation_only_msgs::ObjectInitialization& msg, const ros::Time& timestamp);
    void removeObject(const unsigned int objectId);
    bool containsObjects();
    bool checkObjectExistence(const unsigned int objectId);
    dyn_obj_ptr_t getObjectStateById(const unsigned int objectId);
    std::vector<dyn_obj_ptr_t> getActiveObjectStates();

    automated_driving_msgs::ObjectStateArray activeObjectsToMsg(const ros::Time& timestamp);

private:
    std::string frameId_;
    std::string frameIdObjectsPrefix_;
    ros::Time timestampOfLastUpdate_;
    std::unordered_map<unsigned int, dyn_obj_ptr_t> objectStateMap_;

    void setFrameIds(std::string frameId, std::string frameIdObjectsPrefix);
    void determineActiveStateAndInterpolatePoses(const ros::Time& timestamp);
    std::vector<dyn_obj_ptr_t> getAllObjectStates();
};

automated_driving_msgs::MotionState newMotionStatePoseOnly();

} // namespace localization_mgmt_types
