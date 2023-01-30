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

#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "msceqf/system/sensor_data.hpp"
#include "msceqf/vision/camera.hpp"
#include "msceqf/vision/track.hpp"
#include "types/fptypes.hpp"

namespace msceqf
{

/**
 * @brief All the matches in two subsequent images.
 * A match represent a pair of two features, previous and actual.
 * These two features represent the same feature detected at different time, in subsequent images
 *
 */
struct Matches
{
  std::vector<Feature> previus_;
  std::vector<Feature> actual_;
};

/**
 * @brief This class implement the feature tracker module based on Lucas-Kanade optical flow.
 * The tracker tracks feature temporally in subsequent images and produces a set of matches.
 *
 */
class Tracker
{
 public:
  using Keypoints = std::vector<cv::KeyPoint>;  //!< A vector of keypoints

  /**
   * @brief Tracker constructor
   *
   * @param opts tracker options
   * @param intrinsics camera intrinsics
   */
  Tracker(const TrackerOptions& opts, const Vector4& intrinsics);

  /**
   * @brief This method process the input camera measurement.
   *
   * @param cam
   */
  void processCamera(Camera& cam);

 private:
  void track(Camera& cam);
  void detect(Camera& cam);

  /**
   * @brief This method build a mask for keypoint extraction.
   * Given an existing mask, this method mask out existing keypoints, as well as a small neighborhood to ensure a
   * minimum pixel distance between keypoints
   *
   * @param mask
   */
  void maskPreviouskeypoints(cv::Mat& mask);

  /**
   * @brief Extract keypoints for the given cell. Extracted keypoints are limited to a maximum number given by the
   * difference between the max number of features allowed and the number of previous features.
   *
   * @param cell
   * @param mask
   * @param cell_kpts
   */
  void extractCellKeypoints(const cv::Mat& cell, const cv::Mat& mask, Keypoints& cell_kpts);

  TrackerOptions opts_;              //!< Tracker options
  PinholeCameraUniquePtr cam_;       //!< Pointer to the pinhole camera object
  cv::Ptr<cv::Feature2D> detector_;  //!< The feature detector

  Camera previous_camera_meas_;  //!< Previous frame recorded camera image and mask
  Keypoints previous_kpts_;      //!< Keypoints detected in previous image

  Matches matches_;  //!< The set of mathces between the previous and the actual image

  uint max_kpts_per_cell_;  //!< Maximum number of keypoints for each cell of the grid

  std::vector<cv::Mat> pyramids_;  //!< Pyramids for Optical Flow
  cv::Size win_;                   //!< The Optical Flow window size
};

}  // namespace msceqf

#endif  // TRACKER_HPP

// [TODO] Check constructor init list
// [TODO] Add params to options
// [TODO] Preallocation of vectors
// [TODO] Add support for goodfeaturestotrack GFTT
// [TODO] Subpixel refinement