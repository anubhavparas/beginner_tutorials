/**
 * @file ros_subscriber.cpp
 * @author Anubhav Paras (anubhav@umd.edu)
 * @brief Definitions of the methods of ROSSubscriber class
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <beginner_tutorials/ros_subscriber.hpp>

ROSSubscriber::ROSSubscriber(ros::NodeHandle ros_node_h) {
    this->ros_node_h = ros_node_h;
}

ROSSubscriber::~ROSSubscriber() {}

void ROSSubscriber::chatter_call_back(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Yes, I heard [%s]", msg->data.c_str());
}

void ROSSubscriber::run_subscriber() {
    chatter_sub = this->ros_node_h.subscribe("chatter",
                                             1000,
                                            &ROSSubscriber::chatter_call_back,
                                            this);

    ros::spin();
}


