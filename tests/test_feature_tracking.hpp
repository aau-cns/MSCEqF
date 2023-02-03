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
#include "msceqf/vision/tracker.hpp"
#include "utils/csv_parser.hpp"

namespace msceqf::vision
{

TEST(DetectionTest, fastDetectionNoMask)
{
  std::string config_filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";

  MSCEqF sys(config_filepath_base + "parameters.yaml");
  Tracker tracker(sys.options().tracker_options_, sys.stateOptions().initial_camera_intrinsics_.k());

  Camera cam;
  Tracks tracks;

  cv::Point2f center(320, 240);
  cv::Point2f speed(3, 3);
  int radius = 100;

  int time = -1;

  for (int i = 0; i < 1000; ++i)
  {
    cam.image_ = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
    cam.mask_ = cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);
    cam.timestamp_ = ++time;

    center += speed;

    if (center.x < radius || center.x > cam.image_.cols - radius) speed.x *= -1;
    if (center.y < radius || center.y > cam.image_.rows - radius) speed.y *= -1;

    cv::circle(cam.image_, center, radius, cv::Scalar(255), cv::FILLED);

    tracker.processCamera(cam);
    tracker.updateTracks(tracks);

    Tracker::Keypoints kpts;
    for (auto it = tracks.begin(); it != tracks.end(); ++it)
    {
      auto feat = it->second.back().uvs_.back();

      // Display only active tracks
      if (it->second.back().timestamps_.back() == cam.timestamp_)
      {
        kpts.emplace_back(feat.x, feat.y, 5.0f);
      }
    }
    cv::Mat tmp;
    cv::drawKeypoints(cam.image_, kpts, tmp, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Image with keypoints", tmp);
    cv::waitKey(1);
  }
}

}  // namespace msceqf::vision

#endif  // TEST_DETECTION_HPP