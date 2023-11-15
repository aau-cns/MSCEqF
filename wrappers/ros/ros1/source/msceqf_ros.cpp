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

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "msceqf_ros.hpp"
#include "utils/logger.hpp"

MSCEqFRos::MSCEqFRos(const ros::NodeHandle &nh,
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
                     const std::string &bagfile)
    : nh_(nh), sys_(msceqf_config_filepath)
{
  sub_cam_ = nh_.subscribe(cam_topic, 10, &MSCEqFRos::callback_image, this);
  sub_imu_ = nh_.subscribe(imu_topic, 1000, &MSCEqFRos::callback_imu, this);

  utils::Logger::info("Subscribing: " + std::string(sub_cam_.getTopic().c_str()));
  utils::Logger::info("Subscribing: " + std::string(sub_imu_.getTopic().c_str()));

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>(path_topic, 1);
  pub_image_ = nh_.advertise<sensor_msgs::Image>(image_topic, 1);
  pub_extrinsics_ = nh_.advertise<geometry_msgs::PoseStamped>(extrinsics_topic, 1);
  pub_intrinsics_ = nh_.advertise<sensor_msgs::CameraInfo>(intrinsics_topic, 1);
  pub_origin_ = nh_.advertise<geometry_msgs::PoseStamped>(origin_topic, 1);

  utils::Logger::info("Publishing: " + std::string(pub_pose_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_path_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_image_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_extrinsics_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_intrinsics_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_origin_.getTopic().c_str()));

  record_ = record;
  if (record_)
  {
    bag_.open(bagfile, rosbag::bagmode::Write);
  }

  latest_imu_timestamp_ = -1.0;
}

void MSCEqFRos::callback_image(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    utils::Logger::err("cv_bridge exception: " + std::string(e.what()));
    return;
  }

  msceqf::Camera cam;

  cam.timestamp_ = cv_ptr->header.stamp.toSec();
  cam.image_ = cv_ptr->image.clone();

  cams_.push_back(cam);
  std::sort(cams_.begin(), cams_.end());

  while (!cams_.empty() && cams_.front().timestamp_ < latest_imu_timestamp_)
  {
    sys_.processMeasurement(cams_.front());
    publish(cams_.front());
    cams_.pop_front();
  }
}

void MSCEqFRos::callback_imu(const sensor_msgs::Imu::ConstPtr &msg)
{
  msceqf::Imu imu;

  imu.timestamp_ = msg->header.stamp.toSec();
  imu.ang_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  imu.acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  sys_.processMeasurement(imu);

  latest_imu_timestamp_ = imu.timestamp_;
}

void MSCEqFRos::publish(const msceqf::Camera &cam)
{
  if (!sys_.isInit())
  {
    return;
  }

  auto est = sys_.stateEstimate();
  auto origin = sys_.stateOrigin();

  pose_.header.stamp.fromSec(cam.timestamp_);
  pose_.header.frame_id = "global";
  pose_.header.seq = seq_;

  pose_.pose.pose.orientation.x = est.T().q().x();
  pose_.pose.pose.orientation.y = est.T().q().y();
  pose_.pose.pose.orientation.z = est.T().q().z();
  pose_.pose.pose.orientation.w = est.T().q().w();

  pose_.pose.pose.position.x = est.T().p().x();
  pose_.pose.pose.position.y = est.T().p().y();
  pose_.pose.pose.position.z = est.T().p().z();

  // The covairance is published in the ROS convention order, postion first then orientation
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  cov.block<3, 3>(0, 0) = sys_.coreCovariance().block<3, 3>(6, 6);
  cov.block<3, 3>(0, 3) = sys_.coreCovariance().block<3, 3>(6, 0);
  cov.block<3, 3>(3, 0) = sys_.coreCovariance().block<3, 3>(0, 6);
  cov.block<3, 3>(3, 3) = sys_.coreCovariance().block<3, 3>(0, 0);
  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      pose_.pose.covariance[6 * r + c] = cov(r, c);
    }
  }

  pub_pose_.publish(pose_);

  if (record_)
  {
    bag_.write(pub_pose_.getTopic().c_str(), pose_.header.stamp, pose_);
  }

  if (pub_origin_.getNumSubscribers() != 0)
  {
    origin_.header.stamp.fromSec(cam.timestamp_);
    origin_.header.frame_id = "global";
    origin_.header.seq = seq_;

    origin_.pose.orientation.x = origin.T().q().x();
    origin_.pose.orientation.y = origin.T().q().y();
    origin_.pose.orientation.z = origin.T().q().z();
    origin_.pose.orientation.w = origin.T().q().w();

    origin_.pose.position.x = origin.T().p().x();
    origin_.pose.position.y = origin.T().p().y();
    origin_.pose.position.z = origin.T().p().z();

    pub_origin_.publish(origin_);

    if (record_)
    {
      bag_.write(pub_origin_.getTopic().c_str(), origin_.header.stamp, origin_);
    }
  }

  if (pub_path_.getNumSubscribers() != 0)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = pose_.header;
    pose.pose = pose_.pose.pose;

    path_.header.stamp = ros::Time::now();
    path_.header.seq = seq_;
    path_.header.frame_id = "global";
    path_.poses.push_back(pose);

    pub_path_.publish(path_);
  }

  if (pub_image_.getNumSubscribers() != 0)
  {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "cam0";
    sensor_msgs::ImagePtr img = cv_bridge::CvImage(header, "bgr8", sys_.imageWithTracks(cam)).toImageMsg();
    pub_image_.publish(img);
  }

  if (pub_extrinsics_.getNumSubscribers() != 0)
  {
    extrinsics_.header.stamp.fromSec(cam.timestamp_);
    extrinsics_.header.frame_id = "imu";
    extrinsics_.header.seq = seq_;
    extrinsics_.pose.orientation.x = est.S().q().x();
    extrinsics_.pose.orientation.y = est.S().q().y();
    extrinsics_.pose.orientation.z = est.S().q().z();
    extrinsics_.pose.orientation.w = est.S().q().w();
    extrinsics_.pose.position.x = est.S().x().x();
    extrinsics_.pose.position.y = est.S().x().y();
    extrinsics_.pose.position.z = est.S().x().z();

    pub_extrinsics_.publish(extrinsics_);

    if (record_)
    {
      bag_.write(pub_extrinsics_.getTopic().c_str(), extrinsics_.header.stamp, extrinsics_);
    }
  }

  if (pub_intrinsics_.getNumSubscribers() != 0)
  {
    auto intr = est.k();

    intrinsics_.header.stamp.fromSec(cam.timestamp_);
    intrinsics_.header.frame_id = "cam";
    intrinsics_.header.seq = seq_;
    intrinsics_.height = cam.image_.rows;
    intrinsics_.width = cam.image_.cols;
    intrinsics_.distortion_model = "";
    intrinsics_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    intrinsics_.K = {intr(0), 0.0, intr(2), 0.0, intr(1), intr(3), 0.0, 0.0, 1.0};
    intrinsics_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    intrinsics_.P = {intr(0), 0.0, intr(2), 0.0, 0.0, intr(1), intr(3), 0.0, 0.0, 0.0, 1.0, 0.0};

    pub_intrinsics_.publish(intrinsics_);

    if (record_)
    {
      bag_.write(pub_intrinsics_.getTopic().c_str(), intrinsics_.header.stamp, intrinsics_);
    }
  }

  ++seq_;
}