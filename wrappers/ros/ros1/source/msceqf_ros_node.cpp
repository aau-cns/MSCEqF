// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "msceqf_ros.hpp"

// Main function
int main(int argc, char **argv)
{
  // Launch ros node
  ros::init(argc, argv, "msceqf_ros");
  ros::NodeHandle nh("~");

  // Parameters from launchfile
  std::string config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic, extrinsics_topic,
      intrinsics_topic, origin_topic;

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
    ROS_WARN("Pose topic not defined, using /pose by default");
    pose_topic = "/pose";
  }
  if (!nh.getParam("path_topic", path_topic))
  {
    ROS_WARN("Path topic not defined, using /path by default");
    path_topic = "/path";
  }
  if (!nh.getParam("image_topic", image_topic))
  {
    ROS_WARN("Image topic not defined, using /tracks by default");
    image_topic = "/tracks";
  }
  if (!nh.getParam("extrinsics_topic", extrinsics_topic))
  {
    ROS_WARN("Extrinsics topic not defined, using /extrinsics by default");
    extrinsics_topic = "/extrinsics";
  }
  if (!nh.getParam("intrinsics_topic", intrinsics_topic))
  {
    ROS_WARN("Intrinsics topic not defined, using /intrinsics by default");
    intrinsics_topic = "/intrinsics";
  }
  if (!nh.getParam("origin_topic", origin_topic))
  {
    ROS_WARN("Origin topic not defined, using /origin by default");
    origin_topic = "/origin";
  }

  bool record;
  nh.param("record", record, false);

  std::string outbagfile;
  if (record && !nh.getParam("outbag", outbagfile))
  {
    ROS_ERROR("Recording enabled and output bagfile not defined");
    std::exit(EXIT_FAILURE);
  }

  // Instanciate MSCEqFRos
  MSCEqFRos MSCEqFRos(nh, config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic, extrinsics_topic,
                      intrinsics_topic, origin_topic, record, outbagfile);

  // ROS spin asyncronously
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  ros::shutdown();

  return EXIT_SUCCESS;
}