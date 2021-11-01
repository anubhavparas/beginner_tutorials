/**
 * @file listener.cpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief listener node to listen and subscribe to the topic
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>
#include <beginner_tutorials/ros_subscriber.hpp>


int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle ros_node_h;
  std::unique_ptr<ROSSubscriber> ros_sub(new ROSSubscriber(ros_node_h));
  ros_sub->run_subscriber();
  return 0;
}
