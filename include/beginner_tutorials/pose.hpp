/**
 * @file pose.hpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Struct for storing pose information of a frame
 * @version 0.1
 * @date 2021-11-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_POSE_HPP_
#define SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_POSE_HPP_

struct Pose {
 public:
  /**
   * @brief Construct a new Pose object
   * 
   * @param x 
   * @param y 
   * @param z 
   * @param roll Angle around x (rad)
   * @param pitch Angle around y (rad)
   * @param yaw Angle around z (rad)
   */
  Pose(float x, float y, float z, float roll, float pitch, float yaw) :
        x(x),
        y(y),
        z(z),
        roll(roll),
        pitch(pitch),
        yaw(yaw) {}

  /**
   * @brief Destroy the Pose object
   * 
   */
  ~Pose() {}

  float x, y, z, roll, pitch, yaw;
};

#endif  // SRC_BEGINNER_TUTORIALS_INCLUDE_BEGINNER_TUTORIALS_POSE_HPP_
