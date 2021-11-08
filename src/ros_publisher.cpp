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
#include <beginner_tutorials/StringChange.h>

using beginner_tutorials::StringChange;

ROSPublisher::ROSPublisher(ros::NodeHandle ros_node_h) {
  this->ros_node_h = ros_node_h;
  this->chatter_pub = this->ros_node_h.advertise<std_msgs::String>(
      "chatter", 1000);

  this->modify_str_svc_client = this->ros_node_h.serviceClient<StringChange>(
                                                        "modify_string");
}

ROSPublisher::~ROSPublisher() {
}

std::string ROSPublisher::call_modify_str_svc(std::string input_str) {
  StringChange str_change_srv;
  str_change_srv.request.inputstring = input_str;
  if (this->modify_str_svc_client.call(str_change_srv)) {
    ROS_INFO_STREAM("Service called");
    return static_cast<std::string>(str_change_srv.response.outputstring);
  } else {
    ROS_ERROR_STREAM("Failed to call the service /modify_string");
    return input_str;
  }
}


void ROSPublisher::run_publisher() {
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Hey, I am chatting. I am saying:: " << count;

    msg.data = this->call_modify_str_svc(ss.str());

    ROS_INFO_STREAM("ROSPublisher: Message to be published: "
                    << msg.data.c_str());
    this->chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}
