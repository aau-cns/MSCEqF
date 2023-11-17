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

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include "msceqf_ros.hpp"

// Main function
int main(int argc, char **argv)
{
  // Launch ros node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("msceqf_ros", options);

  // Parameters from launchfile
  std::string config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic, extrinsics_topic,
      intrinsics_topic, origin_topic;

  if (!node->get_parameter<std::string>("config_filepath", config_filepath))
  {
    RCLCPP_ERROR(node->get_logger(), "Configuration filepath not defined");
    std::exit(EXIT_FAILURE);
  }
  if (!node->get_parameter<std::string>("imu_topic", imu_topic))
  {
    RCLCPP_ERROR(node->get_logger(), "Imu topic not defined");
    std::exit(EXIT_FAILURE);
  }
  if (!node->get_parameter<std::string>("cam_topic", cam_topic))
  {
    RCLCPP_ERROR(node->get_logger(), "Camera topic not defined");
    std::exit(EXIT_FAILURE);
  }
  if (!node->get_parameter<std::string>("pose_topic", pose_topic))
  {
    RCLCPP_WARN(node->get_logger(), "Pose topic not defined, using /pose by default");
    pose_topic = "/pose";
  }
  if (!node->get_parameter<std::string>("path_topic", path_topic))
  {
    RCLCPP_WARN(node->get_logger(), "Path topic not defined, using /path by default");
    path_topic = "/path";
  }
  if (!node->get_parameter<std::string>("image_topic", image_topic))
  {
    RCLCPP_WARN(node->get_logger(), "Image topic not defined, using /tracks by default");
    image_topic = "/tracks";
  }
  if (!node->get_parameter<std::string>("extrinsics_topic", extrinsics_topic))
  {
    RCLCPP_WARN(node->get_logger(), "Extrinsics topic not defined, using /extrinsics by default");
    extrinsics_topic = "/extrinsics";
  }
  if (!node->get_parameter<std::string>("intrinsics_topic", intrinsics_topic))
  {
    RCLCPP_WARN(node->get_logger(), "Intrinsics topic not defined, using /intrinsics by default");
    intrinsics_topic = "/intrinsics";
  }
  if (!node->get_parameter<std::string>("origin_topic", origin_topic))
  {
    RCLCPP_WARN(node->get_logger(), "Origin topic not defined, using /origin by default");
    origin_topic = "/origin";
  }

  bool record;
  node->get_parameter_or<bool>("record", record, false);

  std::string outbagfile;
  if (record && !node->get_parameter<std::string>("outbag", outbagfile))
  {
    RCLCPP_ERROR(node->get_logger(), "Recording enabled and output bagfile not defined");
    std::exit(EXIT_FAILURE);
  }

  // Instanciate MSCEqFRos
  MSCEqFRos MSCEqFRos(node, config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic,
                      extrinsics_topic, intrinsics_topic, origin_topic, record, outbagfile);

  // ROS Spin
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}