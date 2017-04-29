#!/bin/bash
echo "Creating folder catkin workspace..."
mkdir catkin_ws
cd catkin_ws
echo "Initializing catkin workspace..."
wstool init
mkdir src
cd src
echo "Cloning packages..."
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
git clone https://git.rwth-aachen.de/spp1835/rosparam_handler.git
git clone https://github.com/fzi-forschungszentrum-informatik/automated_driving_msgs.git
git clone https://github.com/gareth-cross/rviz_satellite.git
git clone https://git.rwth-aachen.de/spp1835/simulation_only_msgs
git clone https://git.rwth-aachen.de/spp1835/simulation_initialization_ros_tool
git clone https://git.rwth-aachen.de/spp1835/simulation_management_ros_tool
git clone https://git.rwth-aachen.de/spp1835/simulation_utils
git clone https://git.rwth-aachen.de/spp1835/sim_sample_perception_ros_tool
git clone https://git.rwth-aachen.de/spp1835/sim_sample_prediction_ros_tool
git clone https://git.rwth-aachen.de/spp1835/sim_sample_planning_ros_tool
git clone https://git.rwth-aachen.de/spp1835/sim_sample_communication_ros_tool
git clone https://git.rwth-aachen.de/spp1835/sim_lanelet
git clone https://git.rwth-aachen.de/spp1835/sim_lanelet_visualization_ros_tool
git clone https://git.rwth-aachen.de/spp1835/object_state_array_rviz_plugin_ros
cd ..
echo "Building packages..."
catkin build
echo "Sourcing build files..."
source devel/setup.bash
