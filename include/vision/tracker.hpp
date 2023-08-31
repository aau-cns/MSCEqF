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

#include <atomic>

#include "sensors/sensor_data.hpp"
#include "types/fptypes.hpp"
#include "vision/camera.hpp"
#include "vision/features.hpp"
#include "vision/track.hpp"

namespace msceqf
{
/**
 * @brief This class implement the feature tracker module based on Lucas-Kanade optical flow.
 * The tracker tracks feature temporally in subsequent images and produces a set of matches.
 *
 */
class Tracker
{
 public:
  using Keypoints = std::vector<cv::KeyPoint>;    //!< A vector of features keypoints
  using TimedFeatures = std::pair<fp, Features>;  //!< Set of features associated with a time

  /**
   * @brief Tracker constructor
   *
   * @param opts Tracker options
   * @param intrinsics Camera intrinsics as R4 vector (fx, fy, cx, cy)
   */
  Tracker(const TrackerOptions& opts, const Vector4& intrinsics);

  /**
   * @brief This method process the input camera measurement.
   * If first pre-process the camera image, and then it tracks features.
   *
   * @param cam Camera measurement
   */
  void processCamera(Camera& cam);

  /**
   * @brief Get the current detected/tracked features
   *
   * @return Current detected/tracked features
   */
  const TimedFeatures& currentFeatures() const;

  /**
   * @brief Get the camera pointer
   *
   * @return Pointer to the camera object
   */
  const PinholeCameraUniquePtr& cam() const;

 private:
  /**
   * @brief Detect/Tracks feature in the given camera measurement.
   * If there are no previously detected features than this method simply detect features in the given image.
   * If there are previously detected features than this method tracks temporally these previously detected/tracked
   * features. Moreover if the number of previously detected/tracked falls under a defined threshold these method
   * performs re-detection in the previous image.
   *
   * @param cam Camera measurement
   */
  void track(Camera& cam);

  /**
   * @brief Detect features based on the selected feature detector.
   * This method detects feature in a image through its pyramids. Each pyramid is split in a grid, and features are
   * extracted for each grid cell in parallel. The number of feature per cell is "dynamic". It starts with the policy
   * that the extraction should be uniform for each cell but the max number of features are "redistributed" if the
   * detection produces few features in some cells.
   * Detected features are stored in original distorted, undistorted, and normalized coordinates.
   * An id is assigned to each feature.
   *
   * @param pyramids Vector of pyramids
   * @param mask Mask for the given image
   * @param features Detected features
   *
   */
  void detect(std::vector<cv::Mat>& pyramids, cv::Mat& mask, Features& features);

  /**
   * @brief Match features between consecutive images using Lukas-Kanade Optical Flow.
   * This methods returns a mask with valid tracked/matched features
   *
   * @param mask Mask with valid tracked/matched features
   */
  void matchKLT(std::vector<uchar>& mask);

  /**
   * @brief Reject outlier using RANSAC
   * This methods returns a mask with valid (inlier) features
   *
   * @param mask Mask with valid tracked/matched features
   */
  void ransac(std::vector<uchar>& mask);

  /**
   * @brief Extract keypoints for the given cell. Extracted keypoints are limited to a maximum number given by the
   * difference between the max number of features allowed and the number of previous features.
   *
   * @param idx Index of the cell
   * @param cell Cell of the grid
   * @param mask Mask for the given cell
   * @param cell_kpts Extracted keypoints for the given cell
   */
  void extractCellKeypoints(const uint& idx, const cv::Mat& cell, const cv::Mat& mask, Keypoints& cell_kpts);

  /**
   * @brief This method build a mask for feature extraction.
   * Given an existing mask, this method mask out given existing features points, as well as a small neighborhood to
   * ensure a minimum pixel distance between features
   *
   * @param mask Mask for the given image
   * @param points Features points to be masked out
   */
  void maskGivenFeatures(cv::Mat& mask, const FeaturesCoordinates& points);

  TrackerOptions opts_;  //!< Tracker options

  PinholeCameraUniquePtr cam_;       //!< Pointer to the pinhole camera object
  cv::Ptr<cv::Feature2D> detector_;  //!< The feature detector

  std::map<uint, std::atomic<uint>> max_kpts_per_cell_;  //!< Maximum number of keypoints for each cell of the grid
  uint id_;                                              //!< Feature id counter

  std::vector<cv::Mat> previous_pyramids_;  //!< Pyramids for Optical Flow and feature extraction from previous image
  cv::Mat previous_mask_;                   //!< Maks from previous image
  TimedFeatures previous_features_;         //!< Features detected in previous image associated to their timestamp

  std::vector<cv::Mat> current_pyramids_;  //!< Pyramids for Optical Flow and feature extraction from current image
  TimedFeatures current_features_;         //!< Features detected in previous image associated to their timestamp

  cv::Size win_;  //!< The Optical Flow window size

  static constexpr std::array<uint, 4> ratio_ = {10, 6, 3, 1};  //!< Ratio of features among pyramid levels
};

}  // namespace msceqf

#endif  // TRACKER_HPP