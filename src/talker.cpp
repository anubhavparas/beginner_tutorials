/**
 * @file talker.cpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief talker node to publish messages to the chatter topic
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <memory>
#include <sstream>

#include <beginner_tutorials/ros_publisher.hpp>


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle ros_node_h;
  std::unique_ptr<ROSPublisher> ros_pub(new ROSPublisher(ros_node_h));
  ros_pub->run_publisher();
  return 0;
}