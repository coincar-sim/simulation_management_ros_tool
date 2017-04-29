#include "localization_mgmt.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "localization_mgmt_node");

    simulation_management_ros_tool::LocalizationMgmt localization_mgmt(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
