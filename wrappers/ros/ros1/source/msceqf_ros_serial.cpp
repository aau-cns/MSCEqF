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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Eigen>

#include "msceqf_ros.hpp"

// Main function
int main(int argc, char **argv)
{
  // Launch ros node
  ros::init(argc, argv, "msceqf_ros");
  ros::NodeHandle nh("~");

  std::string config_filepath, imu_topic, cam_topic, pose_topic, path_topic, image_topic, extrinsics_topic,
      intrinsics_topic, bagfile;
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
  if (!nh.getParam("bag", bagfile))
  {
    ROS_ERROR("bagfile not defined");
    std::exit(EXIT_FAILURE);
  }

  double bag_duration, bag_start;
  if (!nh.getParam("bag_start", bag_start))
  {
    ROS_ERROR("bagfile start time not defined");
    std::exit(EXIT_FAILURE);
  }
  if (!nh.getParam("bag_duration", bag_duration))
  {
    ROS_ERROR("bagfile duration not defined");
    std::exit(EXIT_FAILURE);
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
                      intrinsics_topic, record, outbagfile);

  // Load rosbag
  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);

  // Define views
  rosbag::View view_full;
  rosbag::View view;

  view_full.addQuery(bag);
  ros::Time start_time = view_full.getBeginTime();
  start_time += ros::Duration(bag_start);
  ros::Time end_time = (bag_duration < 0) ? view_full.getEndTime() : start_time + ros::Duration(bag_duration);
  ROS_INFO("time start = %.6f", start_time.toSec());
  ROS_INFO("time end   = %.6f", end_time.toSec());
  view.addQuery(bag, start_time, end_time);
  if (view.size() == 0)
  {
    ROS_ERROR("No messages to play on specified topics.");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // Play rosbag
  double max_camera_time = -1;
  std::vector<rosbag::MessageInstance> msgs;
  for (const rosbag::MessageInstance &msg : view)
  {
    if (!ros::ok()) break;
    if (msg.getTopic() == imu_topic)
    {
      msgs.push_back(msg);
    }
    else if (msg.getTopic() == cam_topic)
    {
      msgs.push_back(msg);
      max_camera_time = std::max(max_camera_time, msg.getTime().toSec());
    }
  }
  ROS_INFO("Read %zu messages!", msgs.size());

  // Loop through messages
  ROS_INFO("Looping messages...");
  std::set<int> used_index;
  for (int m = 0; m < static_cast<int>(msgs.size()); m++)
  {
    if (!ros::ok() || msgs.at(m).getTime() > end_time || msgs.at(m).getTime().toSec() > max_camera_time)
    {
      break;
    }

    if (msgs.at(m).getTime() < start_time)
    {
      continue;
    }

    // Skip messages that we have already used
    if (used_index.find(m) != used_index.end())
    {
      used_index.erase(m);
      continue;
    }

    // IMU processing
    if (msgs.at(m).getTopic() == imu_topic)
    {
      MSCEqFRos.callback_imu(msgs.at(m).instantiate<sensor_msgs::Imu>());
    }

    // Camera processing
    if (msgs.at(m).getTopic() == cam_topic)
    {
      MSCEqFRos.callback_image(msgs.at(m).instantiate<sensor_msgs::Image>());
    }
  }

  // Done!
  return EXIT_SUCCESS;
}