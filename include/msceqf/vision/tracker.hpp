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
  /**
   * @brief ...
   *
   * @param cam
   */
  void track(Camera& cam);

  /**
   * @brief Detect and undistort features based on the selected feature detector.
   * This method detects feature in a image through its pyramids. Each pyramid is split in a grid, and features are
   * extracted for each grid cell in parallel. The number of feature per cell is "dynamic". It starts with the policy
   * that the extraction should be uniform for each cell but the max number of features are "redistributed" if the
   * detection produces few features in some cells.
   *
   * @param pyramids
   * @param current_kpts
   *
   * @note This method undistort detected features but it *does not* normalize them
   */
  void detectAndUndistort(std::vector<cv::Mat>& pyramids, cv::Mat& mask, Keypoints& current_kpts);

  /**
   * @brief ...
   *
   */
  void match();

  /**
   * @brief Extract keypoints for the given cell. Extracted keypoints are limited to a maximum number given by the
   * difference between the max number of features allowed and the number of previous features.
   *
   * @param cell
   * @param mask
   * @param cell_kpts
   */
  void extractCellKeypoints(const cv::Mat& cell, const cv::Mat& mask, Keypoints& cell_kpts);

  /**
   * @brief This method build a mask for keypoint extraction.
   * Given an existing mask, this method mask out existing keypoints, as well as a small neighborhood to ensure a
   * minimum pixel distance between keypoints
   *
   * @param mask
   */
  void maskPreviouskeypoints(cv::Mat& mask);

  /**
   * @brief This method build a mask for keypoint extraction.
   * Given an existing mask, this method mask out given keypoints, as well as a small neighborhood to ensure a
   * minimum pixel distance between keypoints
   *
   * @param mask
   * @param kpts
   * @param
   */
  template <typename T>
  void maskGivenkeypoints(cv::Mat& mask, const T& kpts)
  {
    static_assert(std::is_same_v<T, Keypoints> || std::is_same_v<T, std::vector<cv::Point2f>>);
    int px_dist = std::ceil(opts_.min_px_dist_ / 2);
    for (const auto& kpt : kpts)
    {
      int x;
      int y;
      if constexpr (std::is_same_v<T, Keypoints>)
      {
        x = static_cast<int>(kpt.pt.x);
        y = static_cast<int>(kpt.pt.y);
      }
      else
      {
        x = static_cast<int>(kpt.x);
        y = static_cast<int>(kpt.y);
      }
      int x1 = std::max(0, x - px_dist);
      int y1 = std::max(0, y - px_dist);
      int x2 = std::min(mask.cols - 1, x + px_dist);
      int y2 = std::min(mask.rows - 1, y + px_dist);
      mask(cv::Rect(x1, y1, x2 - x1 + 1, y2 - y1 + 1)) = 0;
    }
  }

  TrackerOptions opts_;  //!< Tracker options

  PinholeCameraUniquePtr cam_;  //!< Pointer to the pinhole camera object

  cv::Ptr<cv::Feature2D> detector_;      //!< The feature detector
  std::atomic<uint> max_kpts_per_cell_;  //!< Maximum number of keypoints for each cell of the grid

  std::vector<cv::Mat> previous_pyramids_;  //!< Pyramids for Optical Flow and feature extraction from previous image
  cv::Mat previous_mask_;                   //!< Maks from previous image
  Keypoints previous_kpts_;                 //!< Keypoints detected in previous image

  std::vector<cv::Mat> current_pyramids_;  //!< Pyramids for Optical Flow and feature extraction from current image
  Keypoints current_kpts;                  //!< Keypoints detected or tracked in current image

  Matches matches_;  //!< The set of mathces between the previous and the actual image

  cv::Size win_;  //!< The Optical Flow window size
};

}  // namespace msceqf

#endif  // TRACKER_HPP

// [TODO] Check constructor init list