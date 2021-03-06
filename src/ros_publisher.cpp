/**
 * MIT License
 *
 * Copyright (c) 2021 Anubhav Paras
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file ros_publisher.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of the methods of ROSPublisher class
 * @version 0.1
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <beginner_tutorials/StringChange.h>

#include <beginner_tutorials/pose.hpp>
#include <beginner_tutorials/ros_publisher.hpp>

using beginner_tutorials::StringChange;

ROSPublisher::ROSPublisher(ros::NodeHandle ros_node_h,
                           int buffer_size) :
                  tf_broadcaster(std::make_shared<ROSTfBroadcaster>()) {
  this->ros_node_h = ros_node_h;
  this->chatter_pub = this->ros_node_h.advertise<std_msgs::String>(
      "chatter", buffer_size);

  this->modify_str_svc_client = this->ros_node_h.serviceClient<StringChange>(
      "modify_string_service");
}

ROSPublisher::~ROSPublisher() {
}

void ROSPublisher::publish_msg(const std::string& msg) {
  std_msgs::String pub_msg;
  pub_msg.data = msg;
  ROS_INFO_STREAM("ROSPublisher: Message to be published: "
                    << pub_msg.data.c_str());

  this->chatter_pub.publish(pub_msg);
}

std::string ROSPublisher::call_modify_str_svc(std::string input_str) {
  StringChange str_change_srv;
  str_change_srv.request.inputstring = input_str;
  if (this->modify_str_svc_client.call(str_change_srv)) {
    ROS_INFO_STREAM("ROSPublisher:: modify_string service called.");
    ROS_DEBUG_STREAM("ROSPublisher:: modify_string service returned: "
                     << str_change_srv.response.outputstring);
    return static_cast<std::string>(str_change_srv.response.outputstring);
  } else {
    ROS_ERROR_STREAM("ROSPublisher:: Failed to call modify_string service");
    return input_str;
  }
}

void ROSPublisher::broadcast_transform() {
  Pose pose(1.0, 1.0, 1.0, 0.0, 0.0, 1.54);
  ROS_INFO_STREAM("Broadcasting a frame: Pose: (x,y,z): ("
                  << pose.x << ","
                  << pose.y << ","
                  << pose.z << "):: "
                  << "rpy: ("
                  << pose.roll << ","
                  << pose.pitch << ","
                  << pose.yaw << ")");
  ROS_INFO_STREAM("Parent frame: /world, Child frame: /talk");
  this->tf_broadcaster->broadcast("world", "talk", pose);
}

void ROSPublisher::run_publisher(int loop_rate_val) {
  ros::Rate loop_rate(loop_rate_val);
  int count = 0;
  while (ros::ok()) {
    std::stringstream ss;
    if (count % 10 == 0) {
      ss << "";
    } else {
      ss << "Hey, I am chatting. I am saying:: " << count;
    }

    ROS_INFO_STREAM("ROSPublisher::run_publisher:"
                    << "calling service to modify string.");

    std::string msg = this->call_modify_str_svc(ss.str());
    this->publish_msg(msg);

    this->broadcast_transform();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}
