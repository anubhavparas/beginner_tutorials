/**
 * @file change_str_svc_node.cpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief service node to run the service modify the input string
 * @version 0.1
 * @date 2021-11-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <beginner_tutorials/modify_string_service.hpp>
#include <memory>

int main(int argc, char **argv) {
  ros::init(argc, argv, "modify_string_service");
  ROS_INFO_STREAM("Initialized /modify_string_service node.");

  ros::NodeHandle ros_node_h;
  std::unique_ptr<ChangeStringService> modify_str_svc(
      new ChangeStringService(ros_node_h,
                              "modify_string_service"));
  ROS_INFO_STREAM("Turning up /modify_string_service.");
  modify_str_svc->turn_up_service();
  return 0;
}
