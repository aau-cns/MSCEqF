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
#include <sensor_msgs/PointCloud.h>
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
   * @param Ros ROS node handle
   * @param msceqf_config_filepath Path of configuration yaml file for the msceqf
   * @param imu_topic IMU topic
   * @param cam_topic Camera topic
   * @param pose_topic Pose topic
   * @param path_topic Path topic
   * @param image_topic Image topic
   * @param extrinsics_topic Extrinsics topic
   * @param intrinsics_topic Intrinsics topic
   * @param record Flag to decide wether record a bagfile or not
   * @param bagfile Bagfile name
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
            const std::string &origin_topic,
            const bool &record,
            const std::string &bagfile);

  /**
   * @brief Image callback
   * @param Message message constant pointer
   */
  void callback_image(const sensor_msgs::Image::ConstPtr &msg);

  /**
   * @brief IMU callback
   * @param Message message constant pointer
   */
  void callback_imu(const sensor_msgs::Imu::ConstPtr &msg);

 private:
  /**
   * @brief Publish pose, images and path messages
   *
   * @param cam Camera measurement
   */
  void publish(const msceqf::Camera &cam);

  ros::NodeHandle nh_;  //!< ROS node handler

  msceqf::MSCEqF sys_;  //!< MSCEqF system

  ros::Subscriber sub_cam_;  //!< Camera subscriber
  ros::Subscriber sub_imu_;  //!< IMU subscriber

  ros::Publisher pub_pose_;        //!< Pose publisher
  ros::Publisher pub_image_;       //!< Image publisher
  ros::Publisher pub_path_;        //!< Path publisher
  ros::Publisher pub_extrinsics_;  //!< Extrinsics publisher
  ros::Publisher pub_intrinsics_;  //!< Intrinsics publisher
  ros::Publisher pub_origin_;      //!< Origin publisher

  geometry_msgs::PoseWithCovarianceStamped pose_;  //!< Pose message
  nav_msgs::Path path_;                            //!< Path message
  geometry_msgs::PoseStamped extrinsics_;          //!< Extrinsics message
  sensor_msgs::CameraInfo intrinsics_;             //!< Intrinsics message
  geometry_msgs::PoseStamped origin_;              //!< Origin message

  bool record_;      //!< Record flag
  rosbag::Bag bag_;  //!< Bagfile

  uint seq_ = 0;  //!< Sequence number
};

#endif  // MSCEQF_ROS_H