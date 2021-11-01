/**
 * @file ros_subscriber.hpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief Class to subscribe and read messages from /chatter topic
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_BEGINNER_TUTORIALS_ROS_SUBSCRIBER_HPP_
#define INCLUDE_BEGINNER_TUTORIALS_ROS_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <string>



/**
 * @brief ROSSubscriber class to publish messages
 * 
 */
class ROSSubscriber {
 public:
  /**
  * @brief Construct a new ROSSubscriber object
  * 
  * @param ros_node_h 
  */
  explicit ROSSubscriber(ros::NodeHandle ros_node_h);

  /**
   * @brief Destroy the ROSSubscriber object
   * 
   */
  virtual ~ROSSubscriber();

  /**
   * @brief to run the publisher node and publish messages at some fixed rate.
   * The loop rate is set to 10
   * The message that is to be sent is also fixed in this case:
   * Message is: "Hey, I am chatting. I am saying:: <count>"
   * 
   */
  virtual void run_subscriber();

 private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle ros_node_h;
  ros::Subscriber chatter_sub;

  /**
   * @brief callback method for the subscriber
   * 
   * @param msg message read by the subsriber
   */
  void chatter_call_back(const std_msgs::String::ConstPtr& msg);
};

#endif  // INCLUDE_BEGINNER_TUTORIALS_ROS_SUBSCRIBER_HPP_
