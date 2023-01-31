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
   * @brief Detect features based on the selected feature detector.
   * This method detects feature in a image splitting the image in a grid, and extracting features for each grid cell in
   * parallel. The number of feature per cell is "dynamic". It starts with the policy that the extraction should be
   * uniform for each cell but the max number of features are "redistributed" if the detection produces few features in
   * some cells.
   *
   * @param cam
   * @param current_kpts
   */
  void detect(Camera& cam, Keypoints& current_kpts);

  /**
   * @brief This method build a mask for keypoint extraction.
   * Given an existing mask, this method mask out existing keypoints, as well as a small neighborhood to ensure a
   * minimum pixel distance between keypoints
   *
   * @param mask
   */
  void maskPreviouskeypoints(cv::Mat& mask);

  /**
   * @brief Remove keypoints which eulidean distance is smaller than min pixel distance using a two pointer approach
   *
   * @param feats Keypoints& or std::vector<cv::Pint2f>&
   *
   * @note This method *does not* ensure that two feature with distance smaller than threshold are removed
   */
  template <typename T>
  void removeCloseFeatures(T& feats)
  {
    static_assert(std::is_same_v<T, Keypoints> || std::is_same_v<T, std::vector<cv::Point2f>>);
    size_t i = 0, j = 0;
    cv::Point2f ref(1, 1);
    if constexpr (std::is_same_v<T, Keypoints>)
    {
      std::sort(feats.begin(), feats.end(),
                [&ref](const cv::KeyPoint& pre, const cv::KeyPoint& post)
                { return pre.pt.dot(ref) < post.pt.dot(ref); });
    }
    else
    {
      std::sort(feats.begin(), feats.end(),
                [&ref](const cv::Point2f& pre, const cv::Point2f& post) { return pre.dot(ref) < post.dot(ref); });
    }
    while (j < feats.size())
    {
      cv::Point2f diff;
      if constexpr (std::is_same_v<T, Keypoints>)
      {
        diff = feats[j].pt - feats[i].pt;
      }
      else
      {
        diff = feats[j] - feats[i];
      }
      if (std::sqrt(diff.dot(diff)) < opts_.min_px_dist_)
      {
        ++j;
      }
      else
      {
        if (i + 1 != j)
        {
          feats[i + 1] = feats[j];
        }
        ++i;
        ++j;
      }
    }
    feats.resize(i + 1);
  }

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

  std::atomic<uint> max_kpts_per_cell_;  //!< Maximum number of keypoints for each cell of the grid

  std::vector<cv::Mat> pyramids_;  //!< Pyramids for Optical Flow
  cv::Size win_;                   //!< The Optical Flow window size
};

}  // namespace msceqf

#endif  // TRACKER_HPP

// [TODO] Check constructor init list