#include "time_mgmt.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "time_mgmt_node");

    simulation_management_ros_tool::TimeMgmt time_mgmt(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();

    return 0;
}
