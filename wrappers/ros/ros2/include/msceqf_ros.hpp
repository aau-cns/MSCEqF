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
#include <atomic>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rosbag2_cpp/writer.hpp>

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
   * @param pose_topic Pose topic
   * @param path_topic Path topic
   * @param image_topic Image topic
   * @param extrinsics_topic Extrinsics topic
   * @param intrinsics_topic Intrinsics topic
   * @param record Flag to record a bagfile
   * @param bagfile Bagfile name
   */
  MSCEqFRos(std::shared_ptr<rclcpp::Node> node,
            const std::string &msceqf_config_filepath,
            const std::string &imu_topic,
            const std::string &cam_topic,
            const std::string &pose_topic,
            const std::string &path_topic,
            const std::string &image_topic,
            const std::string &extrinsics_topic,
            const std::string &intrinsics_topic,
            const std::string &origin_topic,
            const bool &record,
            const std::string &bagfile);

  /**
   * @brief Camera callback
   * @param Message message constant pointer
   */
  void callback_image(const sensor_msgs::msg::Image::SharedPtr &msg);

  /**
   * @brief IMU callback
   * @param Message message constant pointer
   */
  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr &msg);

 private:
  /**
   * @brief Publish pose, images and path messages
   *
   * @param cam Camera measurement
   */
  void publish(const msceqf::Camera &cam);

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
    uint32_t sec = static_cast<uint32_t>(sec64);
    uint32_t nsec = static_cast<uint32_t>(boost::math::round((t - sec) * 1e9));
    sec += (nsec / 1000000000ul);
    nsec %= 1000000000ul;
    return rclcpp::Time(sec, nsec);
  }

  std::shared_ptr<rclcpp::Node> node_;  //<! ROS node

  msceqf::MSCEqF sys_;  //<! MSCEqF system

  rclcpp::Subscription<sensor_msgs::msg::Image::SharedPtr>::SharedPtr sub_cam_;  //<! Camera subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu::SharedPtr>::SharedPtr sub_imu_;    //<! IMU subscriber

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;  //<! Pose publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;                       //<! Image publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;                            //<! Path publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_extrinsics_;          //<! Extrinsics publisher
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_intrinsics_;             //<! Intrinsics publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_origin_;              //<! Origin publisher

  geometry_msgs::msg::PoseWithCovarianceStamped pose_;  //<! Pose message
  nav_msgs::msg::Path path_;                            //<! Path message
  geometry_msgs::msg::PoseStamped extrinsics_;          //<! Extrinsics message
  sensor_msgs::msg::CameraInfo intrinsics_;             //<! Intrinsics message
  geometry_msgs::msg::PoseStamped origin_;              //<! Origin message

  std::deque<msceqf::Camera> cams_;       //!< Camera measurements
  std::mutex mutex_;                      //!< Camera measurements mutex
  std::atomic<bool> processing_ = false;  //!< Camera measurements processing flag

  bool record_;                                      //<! Flag to record a bagfile
  std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;  //<! Bagfile writer
};

#endif  // MSCEQF_ROS_H