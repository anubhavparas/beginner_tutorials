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
 * @file change_str_svc_node.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief service node to run the service modify the input string
 * @version 0.1
 * @date 2021-11-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <beginner_tutorials/modify_string_service.hpp>
#include <memory>

int main(int argc, char **argv) {
  ros::init(argc, argv, "modify_string_service");
  ROS_INFO_STREAM("Initialized /modify_string_service node.");

  ros::NodeHandle ros_node_h;
  std::unique_ptr<ChangeStringService> modify_str_svc(
      new ChangeStringService(ros_node_h,
                              "modify_string_service"));
  ROS_INFO_STREAM("Turning up /modify_string_service.");
  modify_str_svc->turn_up_service();
  return 0;
}
