/**
 * @file modify_string_service.hpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Class to create a service to change the string and append 'Hello'
 * @version 0.1
 * @date 2021-11-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_MODIFY_STRING_SERVICE_HPP_  // NOLINT-CPP
#define SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_MODIFY_STRING_SERVICE_HPP_  // NOLINT-CPP

#include <beginner_tutorials/StringChange.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <string>

class ChangeStringService {
 public:
  /**
   * @brief Construct a new ChangeStringService object
   * 
   * @param ros_node_h 
   * @param service_name name of the service
   */
    ChangeStringService(ros::NodeHandle ros_node_h,
                        const std::string& service_name);

  /**
   * @brief Destroy the Change String Service object
   * 
   */
  ~ChangeStringService();

  /**
   * @brief method to spawn the service up
   * 
   */
  void turn_up_service();

 private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle ros_node_h;
  ros::ServiceServer change_string_svc;
  std::string svc_name;

  /**
   * @brief callback method for the service
   * 
   * @param req wrapper around the the input string to be modified
   * @param res wrapper around the output string
   * @return true if service is success
   * @return false if service fails
   */
  bool modify_string(beginner_tutorials::StringChange::Request& req,  // NOLINT-CPP
                     beginner_tutorials::StringChange::Response& res);  // NOLINT-CPP
};

#endif  // SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_MODIFY_STRING_SERVICE_HPP_  // NOLINT-CPP
