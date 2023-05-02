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

MSCEqFRos::MSCEqFRos(ros::NodeHandle& nh,
                     std::string& msceqf_config_filepath,
                     std::string& imu_topic,
                     std::string& cam_topic,
                     std::string& pose_topic,
                     std::string& path_topic,
                     std::string& image_topic)
    : nh_(nh), sys_(msceqf_config_filepath)
{
  sub_cam_ = nh.subscribe(cam_topic, 1, &MSCEqFRos::callback_image, this);
  sub_imu_ = nh.subscribe(imu_topic, 500, &MSCEqFRos::callback_imu, this);

  utils::Logger::info("Subscribing: " + std::string(sub_cam_.getTopic().c_str()));
  utils::Logger::info("Subscribing: " + std::string(sub_imu_.getTopic().c_str()));

  // Publishers
  pub_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1);
  pub_path_ = nh.advertise<nav_msgs::Path>(path_topic, 1);
  pub_image_ = nh.advertise<sensor_msgs::Image>(image_topic, 1);

  // Print topics where we are publishing on
  utils::Logger::info("Publishing: " + std::string(pub_pose_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_path_.getTopic().c_str()));
  utils::Logger::info("Publishing: " + std::string(pub_image_.getTopic().c_str()));
}

void MSCEqFRos::callback_image(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    utils::Logger::err("cv_bridge exception: " + std::string(e.what()));
    return;
  }

  msceqf::Camera cam;

  cam.timestamp_ = cv_ptr->header.stamp.toSec();
  cam.image_ = cv_ptr->image.clone();
  cam.mask_ = 255 * cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);

  sys_.processMeasurement(cam);

  publish(cam);
}

void MSCEqFRos::callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  msceqf::Imu imu;

  imu.timestamp_ = msg->header.stamp.toSec();
  imu.ang_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  imu.acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  sys_.processMeasurement(imu);
}

void MSCEqFRos::publish(const msceqf::Camera& cam)
{
  if (!sys_.isInit())
  {
    return;
  }

  auto est = sys_.stateEstimate();

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
  cov.block<3, 3>(0, 0) = sys_.covariance().block<3, 3>(3, 3);
  cov.block<3, 3>(0, 3) = sys_.covariance().block<3, 3>(3, 0);
  cov.block<3, 3>(3, 0) = sys_.covariance().block<3, 3>(0, 3);
  cov.block<3, 3>(3, 3) = sys_.covariance().block<3, 3>(0, 0);
  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      pose_.pose.covariance[6 * r + c] = cov(r, c);
    }
  }

  pub_pose_.publish(pose_);

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

  ++seq_;
}
