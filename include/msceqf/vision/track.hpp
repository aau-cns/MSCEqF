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

#ifndef TRACK_HPP
#define TRACK_HPP

#include <opencv2/opencv.hpp>

#include "msceqf/vision/features.hpp"
#include "types/fptypes.hpp"

namespace msceqf::vision
{
/**
 * @brief (Cache friendly) Track struct. Define a feature (labelled via a feature id) detected/tracked at different
 * points in time.
 *
 * @note Note that feature cordinates are of cv::Point2f type for compatibility with OpenCV.
 *
 */
struct Track
{
  using Times = std::vector<fp>;  //!< vector of timestamps

  /**
   * @brief Check if there valid coordinates in uvs_
   *
   * @return true if coordinates are found, false otherwise
   */
  inline bool empty() const noexcept { return uvs_.empty(); }

  /**
   * @brief Return the amount of features (size of uvs_)
   *
   * @return size_t
   */
  inline size_t size() const noexcept
  {
    assert(uvs_.size() == normalized_uvs_.size());
    assert(uvs_.size() == timestamps_.size());
    return uvs_.size();
  }

  /**
   * @brief Remove invalid features coordinates, normalized feature coordinates and ids given a vector of boolean flags
   * indicating invalid features
   *
   * @param invalid
   */
  void remove(std::vector<bool>& invalid)
  {
    assert(invalid.size() == uvs_.size());
    assert(uvs_.size() == normalized_uvs_.size());
    assert(uvs_.size() == timestamps_.size());

    uvs_.erase(std::remove_if(uvs_.begin(), uvs_.end(),
                              [&invalid, this](const cv::Point2f& feat) { return invalid[&feat - &uvs_[0]]; }),
               uvs_.end());

    normalized_uvs_.erase(
        std::remove_if(normalized_uvs_.begin(), normalized_uvs_.end(),
                       [&invalid, this](const cv::Point2f& feat) { return invalid[&feat - &normalized_uvs_[0]]; }),
        normalized_uvs_.end());

    timestamps_.erase(
        std::remove_if(timestamps_.begin(), timestamps_.end(),
                       [&invalid, this](const fp& timestamp) { return invalid[&timestamp - &timestamps_[0]]; }),
        timestamps_.end());
  }

  /**
   * @brief Comparison operator with other tracks for sorting based on track length
   *
   */
  friend bool operator<(const Track& lhs, const Track& rhs) { return lhs.size() < rhs.size(); }

  FeaturesCoordinates uvs_;             //!< (u, v) coordinates of the same feature at different time steps
  FeaturesCoordinates normalized_uvs_;  //!< Normalized (u, v) coordinates of the same feature at different time steps
  Times timestamps_;                    //!< Timestamps of the camera measurement containing the feature
};

using Tracks = std::unordered_map<uint, std::vector<Track>>;  //!< Tracks defined as a a vector of tracks mapped by ids

}  // namespace msceqf::vision

#endif  // TRACK_HPP