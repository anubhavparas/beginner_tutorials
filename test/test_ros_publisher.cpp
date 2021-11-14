/**
 * @file test_ros_publisher.cpp
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief To test the ros_publisher class
 * @version 0.1
 * @date 2021-11-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <beginner_tutorials/ros_publisher.hpp>

struct HelperCallback {
 public:
  void callback(const std_msgs::String::ConstPtr& msg) {
    message = msg->data.c_str();
  }
  std::string message;
};

TEST(ROSPublisherTest, testPublishMessage) {
  ros::NodeHandle ros_node_h;
  HelperCallback cb;

  // this is going to subsribe to the /chatter topic
  ros::Subscriber ros_sub = ros_node_h.subscribe("chatter",
                                      10,
                                      &HelperCallback::callback,
                                      &cb);

  // wait for the publisher node to be up and broadcast tf frame
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  std::string substring_msg = "Hello";
  std::string subscribed_msg = cb.message;

  EXPECT_TRUE(subscribed_msg.find(substring_msg) != std::string::npos);
}

TEST(ROSPublisherTest, testTfBroadcaster) {
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

  // wait for the publisher node to be up and broadcast tf frame
  ros::Duration(2.0).sleep();
  try {
    tf_listener.lookupTransform("/world", "/talk",
                                ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR_STREAM(ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Matrix3x3 rot_mat(transform.getRotation());
  double roll, pitch, yaw;
  rot_mat.getRPY(roll, pitch, yaw);

  // check for translation
  EXPECT_NEAR(transform.getOrigin().x(), 1.0, 0.0001);
  EXPECT_NEAR(transform.getOrigin().y(), 1.0, 0.0001);
  EXPECT_NEAR(transform.getOrigin().z(), 1.0, 0.0001);

  // check for rotation
  EXPECT_NEAR(roll, 0.0, 0.0001);
  EXPECT_NEAR(pitch, 0.0, 0.0001);
  EXPECT_NEAR(yaw, 1.54, 0.0001);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "talker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
