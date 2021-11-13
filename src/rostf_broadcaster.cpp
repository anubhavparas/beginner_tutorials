/**
 * @file rostf_broadcaster.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of class ROSTfBroadcaster 
 * @version 0.1
 * @date 2021-11-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <beginner_tutorials/rostf_broadcaster.hpp>
#include <beginner_tutorials/pose.hpp>

ROSTfBroadcaster::ROSTfBroadcaster() {
}

ROSTfBroadcaster::~ROSTfBroadcaster() {}

void ROSTfBroadcaster::broadcast(
            const std::string& parent_frame,
            const std::string& child_frame,
            const Pose& pose) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.x, pose.y, pose.z));

    tf::Quaternion q;
    q.setRPY(pose.roll, pose.pitch, pose.yaw);
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(
            transform,
            ros::Time::now(),
            parent_frame,
            child_frame));
}
