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
 * @file modify_string_service.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Class to create a service to change the string and append 'Hello'
 * @version 0.1
 * @date 2021-11-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <beginner_tutorials/modify_string_service.hpp>
#include <beginner_tutorials/StringChange.h>


ChangeStringService::ChangeStringService(ros::NodeHandle ros_node_h,
                                         const std::string& service_name) :
                                         svc_name(service_name) {
    this->ros_node_h = ros_node_h;
}
ChangeStringService::~ChangeStringService() {
}


bool ChangeStringService::modify_string(
                    beginner_tutorials::StringChange::Request& req,
                    beginner_tutorials::StringChange::Response& res) {
    ROS_DEBUG_STREAM("ChangeStringService::modify_string::request: "
                    << req.inputstring);

    if (req.inputstring.length() < 1) {
        ROS_WARN_STREAM("ChangeStringService::modify_string: "
                        << "Request has empty string. "
                        << req.inputstring);
    }

    res.outputstring = "Hello World. " + req.inputstring;
    ROS_DEBUG_STREAM("ChangeStringService::modify_string::response: "
                    << res.outputstring);
    return true;
}

void ChangeStringService::turn_up_service() {
    this->change_string_svc = this->ros_node_h.advertiseService(this->svc_name,
                &ChangeStringService::modify_string, this);
    ROS_INFO_STREAM("ChangeStringService::turn_up_service:: "
                    << this->svc_name
                    << " service is up.");
    ros::spin();
}
