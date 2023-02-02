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
  using FeaturesPoints = std::vector<Feature::FeatureCoordinates>;  //!< A vector of features coordinates
  using Keypoints = std::vector<cv::KeyPoint>;                      //!< A vector of features keypoints

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
   * detection produces few features in some cells. After detection the feature ar undistorted and normalized if the
   * normalized argument is set
   *
   *
   * @param pyramids
   * @param mask
   * @param current_feature
   * @param normalize
   *
   * @note This method undistort detected features but it *does not* normalize them
   */
  void detectAndUndistort(std::vector<cv::Mat>& pyramids,
                          cv::Mat& mask,
                          FeaturesPoints& current_feature,
                          const bool& normalize = false);

  /**
   * @brief ...
   *
   */
  void matchKLT(FeaturesPoints& previous_features, FeaturesPoints& current_features, std::vector<uchar>& status);

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
   * @brief This method build a mask for feature extraction.
   * Given an existing mask, this method mask out given existing features points, as well as a small neighborhood to
   * ensure a minimum pixel distance between features
   *
   * @param mask
   * @param points
   */
  void maskGivenFeatures(cv::Mat& mask, const FeaturesPoints& points);

  TrackerOptions opts_;  //!< Tracker options

  PinholeCameraUniquePtr cam_;  //!< Pointer to the pinhole camera object

  cv::Ptr<cv::Feature2D> detector_;      //!< The feature detector
  std::atomic<uint> max_kpts_per_cell_;  //!< Maximum number of keypoints for each cell of the grid
  uint ids_;                             //!< Feature id counter

  std::vector<cv::Mat> previous_pyramids_;  //!< Pyramids for Optical Flow and feature extraction from previous image
  cv::Mat previous_mask_;                   //!< Maks from previous image
  FeaturesPoints previous_features_;        //!< Features detected in previous image

  std::vector<cv::Mat> current_pyramids_;  //!< Pyramids for Optical Flow and feature extraction from current image
  FeaturesPoints current_features_;        //!< Features detected or tracked in current image

  cv::Size win_;  //!< The Optical Flow window size
};

}  // namespace msceqf

#endif  // TRACKER_HPP

// [TODO] Check constructor init list
// [TODO] Implement 2-point RANSAC