/**
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
                                         std::string service_name) {
    this->ros_node_h = ros_node_h;
    this->svc_name = service_name;
}
ChangeStringService::~ChangeStringService() {
}


bool ChangeStringService::modify_string(
                    beginner_tutorials::StringChange::Request& req,
                    beginner_tutorials::StringChange::Response& res) {
    ROS_INFO_STREAM("ChangeStringService::modify_string::request: "
                    << req.inputstring);

    if (req.inputstring.length() < 1) {
        ROS_WARN_STREAM("ChangeStringService::modify_string: "
                        << "Request has empty string. "
                        << req.inputstring);
    }

    res.outputstring = "Hello World. " + req.inputstring;
    ROS_INFO_STREAM("ChangeStringService::modify_string::response: "
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
