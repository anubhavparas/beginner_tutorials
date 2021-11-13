/**
 * @file rostf_broadcaster.hpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Class to broadcast frame information
 * @version 0.1
 * @date 2021-11-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_ROSTF_BROADCASTER_HPP_  //  NOLINT-CPP
#define SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_ROSTF_BROADCASTER_HPP_  //  NOLINT-CPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <beginner_tutorials/pose.hpp>

class ROSTfBroadcaster {
 public:
  /**
   * @brief Construct a new ROSTfBroadcaster object
   * 
   */
  ROSTfBroadcaster();

  /**
   * @brief Destroy the ROSTfBroadcaster object
   * 
   */
  ~ROSTfBroadcaster();

  /**
   * @brief 
   * 
   * @param parent_frame 
   * @param child_frame 
   */
  virtual void broadcast(
            const std::string& parent_frame,
            const std::string& child_frame,
            const Pose& pose);
};

#endif  // SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_ROSTF_BROADCASTER_HPP_  //  NOLINT-CPP