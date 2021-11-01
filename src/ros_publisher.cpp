/**
 * @file ros_publisher.cpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief Definitions of the methods of ROSPublisher class
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <beginner_tutorials/ros_publisher.hpp>

ROSPublisher::ROSPublisher(ros::NodeHandle ros_node_h) {
  this->ros_node_h = ros_node_h;
  this->chatter_pub = this->ros_node_h.advertise<std_msgs::String>(
      "chatter", 1000);
}

ROSPublisher::~ROSPublisher() {
}


void ROSPublisher::run_publisher() {
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Hey, I am chatting. I am saying:: " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    this->chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}
