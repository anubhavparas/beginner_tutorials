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
