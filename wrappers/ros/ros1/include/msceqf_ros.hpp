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

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>

#include "msceqf/msceqf.hpp"

class MSCEqFRos
{
 public:
  /**
   * @brief Constructor
   * @param Ros NodeHandle
   * @param msceqf_config_filepath path of configuration yaml file for the msceqf
   * @param imu_topic
   * @param cam_topic
   * @param pose_topic
   * @param path_topic
   * @param image_topic
   * @param extrinsics_topic
   * @param intrinsics_topic
   * @param record
   * @param bagfile
   */
  MSCEqFRos(const ros::NodeHandle &nh,
            const std::string &msceqf_config_filepath,
            const std::string &imu_topic,
            const std::string &cam_topic,
            const std::string &pose_topic,
            const std::string &path_topic,
            const std::string &image_topic,
            const std::string &extrinsics_topic,
            const std::string &intrinsics_topic,
            const bool &record,
            const std::string &bagfile);

  /**
   * @brief Callbacks
   * @param Message const pointer
   */
  void callback_image(const sensor_msgs::Image::ConstPtr &msg);
  void callback_imu(const sensor_msgs::Imu::ConstPtr &msg);

 private:
  /**
   * @brief Publish pose, images and path messages
   *
   * @param cam
   */
  void publish(const msceqf::Camera &cam);

  /// Ros node handler
  ros::NodeHandle nh_;

  /// MSCEqF
  msceqf::MSCEqF sys_;

  /// Subscribers
  ros::Subscriber sub_cam_;
  ros::Subscriber sub_imu_;

  /// Publishers
  ros::Publisher pub_pose_;
  ros::Publisher pub_image_;
  ros::Publisher pub_path_;
  ros::Publisher pub_extrinsics_;
  ros::Publisher pub_intrinsics_;

  /// Messages
  geometry_msgs::PoseWithCovarianceStamped pose_;
  nav_msgs::Path path_;
  geometry_msgs::PoseStamped extrinsics_;
  sensor_msgs::CameraInfo intrinsics_;

  /// Record bagfile
  bool record_;
  rosbag::Bag bag_;

  /// Sequence number
  uint seq_ = 0;
};

#endif  // MSCEQF_ROS_H