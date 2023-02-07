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

#ifndef TEST_TRACKING_HPP
#define TEST_TRACKING_HPP

#include "msceqf/msceqf.hpp"
#include "utils/csv_parser.hpp"
#include "vision/track_manager.hpp"

namespace msceqf
{

void track(Camera& cam,
           TrackManager& track_manager,
           Tracker::Keypoints& active_kpts,
           Tracker::Keypoints& lost_kpts,
           std::unordered_set<uint>& active_tracks,
           std::unordered_set<uint>& lost_tracks)
{
  const auto& tracks = track_manager.tracks();
  track_manager.tracksIds(cam.timestamp_, active_tracks, lost_tracks);

  for (auto& id : active_tracks)
  {
    active_kpts.emplace_back(tracks.at(id).uvs_.back().x, tracks.at(id).uvs_.back().y, 5.0f);
  }

  for (auto& id : lost_tracks)
  {
    lost_kpts.emplace_back(tracks.at(id).uvs_.back().x, tracks.at(id).uvs_.back().y, 5.0f);
  }

  track_manager.removeTracksId(lost_tracks);

  cv::drawKeypoints(cam.image_, active_kpts, cam.image_, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::drawKeypoints(cam.image_, lost_kpts, cam.image_, cv::Scalar(0, 255, 0),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow("Image with keypoints", cam.image_);
  cv::waitKey(1);

  active_kpts.clear();
  lost_tracks.clear();
  active_tracks.clear();
}

void drawCircle(Camera& cam, int& radius, cv::Point2f& center, cv::Point2f& speed)
{
  center += speed;

  if (center.x < radius || center.x > cam.image_.cols - radius) speed.x *= -1;
  if (center.y < radius || center.y > cam.image_.rows - radius) speed.y *= -1;

  cv::circle(cam.image_, center, radius, cv::Scalar(255), cv::FILLED);
}

TEST(TrackingTest, TrackingNoMask)
{
  std::string config_filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";

  MSCEqF sys(config_filepath_base + "parameters.yaml");

  TrackManager track_manager(sys.options().track_manager_options_, sys.stateOptions().initial_camera_intrinsics_.k());

  Camera cam;
  cv::Size size(640, 480);

  cv::Point2f center(320, 240);
  cv::Point2f speed(3, 3);
  int radius = 100;

  int time = -1;

  Tracker::Keypoints active_kpts, lost_kpts;
  std::unordered_set<uint> active_tracks, lost_tracks;

  for (int i = 0; i < 1000; ++i)
  {
    cam.image_ = cv::Mat::zeros(size, CV_8UC1);
    cam.mask_ = cv::Mat::ones(size, CV_8UC1);
    cam.timestamp_ = ++time;

    drawCircle(cam, radius, center, speed);
    track_manager.processCamera(cam);
    track(cam, track_manager, active_kpts, lost_kpts, active_tracks, lost_tracks);
  }
}

TEST(TrackingTest, TrackingMask)
{
  std::string config_filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";

  MSCEqF sys(config_filepath_base + "parameters.yaml");

  TrackManager track_manager(sys.options().track_manager_options_, sys.stateOptions().initial_camera_intrinsics_.k());

  Camera cam;
  cv::Size size(640, 480);

  cv::Point2f center(320, 240);
  cv::Point2f speed(3, 3);
  int radius = 100;

  int time = -1;

  Tracker::Keypoints active_kpts, lost_kpts;
  std::unordered_set<uint> active_tracks, lost_tracks;

  for (int i = 0; i < 1000; ++i)
  {
    cam.image_ = cv::Mat::zeros(size, CV_8UC1);
    cam.mask_ = cv::Mat::ones(size, CV_8UC1);
    cam.mask_(cv::Rect(320, 0, 320, 480)) = 0;
    cam.timestamp_ = ++time;

    drawCircle(cam, radius, center, speed);
    track_manager.processCamera(cam);
    track(cam, track_manager, active_kpts, lost_kpts, active_tracks, lost_tracks);
  }
}

}  // namespace msceqf

#endif  // TEST_TRACKING_HPP