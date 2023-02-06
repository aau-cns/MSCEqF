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

#ifndef TEST_DETECTION_HPP
#define TEST_DETECTION_HPP

#include "msceqf/msceqf.hpp"
#include "vision/track_manager.hpp"
#include "utils/csv_parser.hpp"

namespace msceqf
{

TEST(DetectionTest, fastDetectionNoMask)
{
  std::string config_filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";

  MSCEqF sys(config_filepath_base + "parameters.yaml");

  TrackManager track_manager(sys.options().track_manager_options_, sys.stateOptions().initial_camera_intrinsics_.k());

  Camera cam;
  Tracks tracks;

  cv::Point2f center(320, 240);
  cv::Point2f speed(3, 3);
  int radius = 100;

  int time = -1;

  Tracker::Keypoints active_kpts, lost_kpts;
  Tracks active_tracks, lost_tracks;
  std::unordered_set<uint> ids_to_remove;

  for (int i = 0; i < 1000; ++i)
  {
    cam.image_ = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
    cam.mask_ = cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);
    cam.timestamp_ = ++time;

    center += speed;

    if (center.x < radius || center.x > cam.image_.cols - radius) speed.x *= -1;
    if (center.y < radius || center.y > cam.image_.rows - radius) speed.y *= -1;

    cv::circle(cam.image_, center, radius, cv::Scalar(255), cv::FILLED);

    track_manager.processCamera(cam);
    track_manager.tracksAt(cam.timestamp_, active_tracks, lost_tracks);

    for (auto& [id, active_track] : active_tracks)
    {
      active_kpts.emplace_back(active_track.uvs_.back().x, active_track.uvs_.back().y, 5.0f);
    }

    for (auto& [id, lost_track] : lost_tracks)
    {
      lost_kpts.emplace_back(lost_track.uvs_.back().x, lost_track.uvs_.back().y, 5.0f);
      ids_to_remove.insert(id);
    }

    track_manager.removeTracksId(ids_to_remove);

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
}

}  // namespace msceqf

#endif  // TEST_DETECTION_HPP