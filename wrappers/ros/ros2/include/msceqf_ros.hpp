// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#ifndef MSCEQF_ROS_H
#define MSCEQF_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <sensor_msgs/msg/Image.h>
#include <sensor_msgs/msg/Imu.h>
#include <sensor_msgs/msg/CameraInfo.h>
#include <sensor_msgs/msg/PointCloud.h>
#include <geometry_msgs/msg/PoseWithCovarianceStamped.h>
#include <nav_msgs/msg/Path.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>

#include "msceqf/msceqf.hpp"

class MSCEqFRos
{
 public:
  /**
   * @brief Constructor
   * @param node Node
   * @param msceqf_config_filepath Path of configuration yaml file for the msceqf
   * @param imu_topic IMU topic
   * @param cam_topic Camera topic
   * @param features_topic Features topic
   * @param pose_topic Pose topic
   * @param path_topic Path topic
   * @param image_topic Image topic
   * @param extrinsics_topic Extrinsics topic
   * @param intrinsics_topic Intrinsics topic
   * @param record Flag to record a bagfile
   * @param bagfile Bagfile name
   */
  MSCEqFRos(const rclcpp::Node &node,
            const std::string &msceqf_config_filepath,
            const std::string &imu_topic,
            const std::string &cam_topic,
            const std::string &features_topic,
            const std::string &pose_topic,
            const std::string &path_topic,
            const std::string &image_topic,
            const std::string &extrinsics_topic,
            const std::string &intrinsics_topic,
            const std::string &origin_topic,
            const bool &record,
            const std::string &bagfile);

  /**
   * @brief Callbacks
   * @param Message const pointer
   */
  void callback_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
  void callback_feats(const sensor_msgs::msg::PointCloud::ConstSharedPtr &msg);

 private:
  /**
   * @brief Publish pose, images and path messages
   *
   * @param cam Camera measurement
   */
  void publish(const msceqf::Camera &cam);

  /**
   * @brief Publish pose, images and path messages
   *
   * @param feats Features measurement
   */
  void publish(const msceqf::TriangulatedFeatures &feats);

  /**
   * @brief Convert time in seconds to rclcpp::Time
   *
   * @param t Time in seconds
   * @return rclcpp time
   *
   * @note Implementation taken from ROS1
   */
  static rclcpp::Time fromSec(double t)
  {
    int64_t sec64 = static_cast<int64_t>(floor(t));
    if (sec64 < 0 || sec64 > std::numeric_limits<uint32_t>::max())
      throw std::runtime_error("Time is out of dual 32-bit range");
    sec = static_cast<uint32_t>(sec64);
    nsec = static_cast<uint32_t>(boost::math::round((t - sec) * 1e9));
    sec += (nsec / 1000000000ul);
    nsec %= 1000000000ul;
    return rclcpp::Time(sec, nsec);
  }

  rclcpp::Node node_;  //<! ROS node

  msceqf::MSCEqF sys_;  //<! MSCEqF system

  rclcpp::Subscription<sensor_msgs::msg::Image> sub_cam_;         //<! Camera subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu> sub_imu_;           //<! IMU subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud> sub_feats_;  //<! Features subscriber

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped> pub_pose_;  //<! Pose publisher
  rclcpp::Publisher<sensor_msgs::msg::Image> pub_image_;                       //<! Image publisher
  rclcpp::Publisher<nav_msgs::msg::Path> pub_path_;                            //<! Path publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped> pub_extrinsics_;          //<! Extrinsics publisher
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo> pub_intrinsics_;             //<! Intrinsics publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped> pub_origin_;              //<! Origin publisher

  geometry_msgs::msg::PoseWithCovarianceStamped pose_;  //<! Pose message
  nav_msgs::msg::Path path_;                            //<! Path message
  geometry_msgs::msg::PoseStamped extrinsics_;          //<! Extrinsics message
  sensor_msgs::msg::CameraInfo intrinsics_;             //<! Intrinsics message
  geometry_msgs::msg::PoseStamped origin_;              //<! Origin message

  bool record_;      //<! Flag to record a bagfile
  rosbag::Bag bag_;  //<! Bagfile

  uint seq_ = 0;  //<! Sequence number
};

#endif  // MSCEQF_ROS_H