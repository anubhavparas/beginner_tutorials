/**
 * @file ros_publisher.hpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief Class to publish fixed message over /chatter topic
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_BEGINNER_TUTORIALS_ROS_PUBLISHER_HPP_
#define INCLUDE_BEGINNER_TUTORIALS_ROS_PUBLISHER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <string>



/**
 * @brief ROSPublisher class to publish messages
 * 
 */
class ROSPublisher {
 public:
  /**
  * @brief Construct a new ROSPublisher object
  * 
  * @param ros_node_h 
  */
  explicit ROSPublisher(ros::NodeHandle ros_node_h);

  /**
   * @brief Destroy the ROSPublisher object
   * 
   */
  virtual ~ROSPublisher();

  /**
   * @brief to run the publisher node and publish messages at some fixed rate.
   * The loop rate is set to 10
   * The message that is to be sent is also fixed in this case:
   * Message is: "Hey, I am chatting. I am saying:: <count>"
   * 
   */
  virtual void run_publisher();

 private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle ros_node_h;
  ros::Publisher chatter_pub;
};

#endif  // INCLUDE_BEGINNER_TUTORIALS_ROS_PUBLISHER_HPP_
