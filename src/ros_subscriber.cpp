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
 * @file ros_subscriber.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
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
    if (msg == nullptr) {
        ROS_FATAL_STREAM(
            "ROSSubscriber::chatter_call_back: msg pointer is null");
    }
    ROS_INFO_STREAM("Yes, I heard: "
                    << "["
                    << msg->data.c_str()
                    << "]");
}

void ROSSubscriber::run_subscriber() {
    chatter_sub = this->ros_node_h.subscribe("chatter",
                                             1000,
                                            &ROSSubscriber::chatter_call_back,
                                            this);

    ros::spin();
}


