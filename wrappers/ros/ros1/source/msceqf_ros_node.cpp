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

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "msceqf_ros");
  ros::NodeHandle nh("~");

  // Parameters from launchfile
  std::string config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic;

  // Check existance of parameter
  if (!nh.getParam("config_filepath", config_filepath))
  {
    ROS_ERROR("Configuration filepath not defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("imu_topic", imu_topic))
  {
    ROS_ERROR("Imu topic not defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("cam_topic", cam_topic))
  {
    ROS_ERROR("Camera topic not defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("pose_topic", pose_topic))
  {
    ROS_ERROR("Pose topic not defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("path_topic", path_topic))
  {
    ROS_ERROR("Pose topic not defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("image_topic", image_topic))
  {
    ROS_ERROR("Pose topic not defined");
    std::exit(EXIT_FAILURE);
  }

  // Instanciate MSCEqFRos
  MSCEqFRos MSCEqFRos(nh, config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic);

  // ROS Spin
  ros::spin();

  // Done!
  return EXIT_SUCCESS;
}