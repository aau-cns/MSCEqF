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

namespace msceqf
{

TEST(DetectionTest, fastDetectionNoMask)
{
  std::string config_filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";
  std::string image_filepath = "/home/alfornasier/Pictures/DSC_1872.jpg";

  MSCEqF sys(config_filepath_base + "parameters.yaml");

  cv::Mat img = cv::imread(image_filepath);
  cv::resize(img, img, cv::Size(640, 480), cv::INTER_LINEAR);

  Camera cam;
  cam.image_ = img;
  cam.mask_ = cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);

  Tracker tracker(sys.options().tracker_options_, sys.stateOptions().initial_camera_intrinsics_.k());
  tracker.processCamera(cam);
}

TEST(DetectionTest, fastDetectionMask)
{
  std::string config_filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";
  std::string image_filepath = "/home/alfornasier/Pictures/DSC_1872.jpg";

  MSCEqF sys(config_filepath_base + "parameters.yaml");

  cv::Mat img = cv::imread(image_filepath);
  cv::resize(img, img, cv::Size(640, 480), cv::INTER_LINEAR);

  Camera cam;
  cam.image_ = img;
  cam.mask_ = cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);
  cam.mask_(cv::Rect(160, 120, 320, 240)) = 0;

  Tracker tracker(sys.options().tracker_options_, sys.stateOptions().initial_camera_intrinsics_.k());
  tracker.processCamera(cam);
}

}  // namespace msceqf

#endif  // TEST_DETECTION_HPP