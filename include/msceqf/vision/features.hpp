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

#ifndef FEATURES_HPP
#define FEATURES_HPP

#include <opencv2/opencv.hpp>

#include "types/fptypes.hpp"

namespace msceqf::vision
{

using FeaturesCoordinates = std::vector<cv::Point2f>;  //!< The features coordinates

/**
 * @brief (Cache friendly) Features struct. Define a set of features detected/tracked.
 *
 * @note Note that feature cordinates are of cv::Point2f type for compatibility with OpenCV.
 *
 */
struct Features
{
  using FeatureIds = std::vector<uint>;  //!< Vector of feature ids

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
    assert(uvs_.size() == ids_.size());
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
    assert(uvs_.size() == ids_.size());

    uvs_.erase(std::remove_if(uvs_.begin(), uvs_.end(),
                              [&invalid, this](const cv::Point2f& feat) { return invalid[&feat - &uvs_[0]]; }),
               uvs_.end());

    normalized_uvs_.erase(
        std::remove_if(normalized_uvs_.begin(), normalized_uvs_.end(),
                       [&invalid, this](const cv::Point2f& feat) { return invalid[&feat - &normalized_uvs_[0]]; }),
        normalized_uvs_.end());

    ids_.erase(
        std::remove_if(ids_.begin(), ids_.end(), [&invalid, this](const uint& id) { return invalid[&id - &ids_[0]]; }),
        ids_.end());
  }

  FeaturesCoordinates uvs_;             //!< (u, v) coordinates of the features detected/tracked
  FeaturesCoordinates normalized_uvs_;  //!< Normalized (u, v) coordinates of features detected/tracked
  FeatureIds ids_;                      //!< Id of the features detected/tracked
};

}  // namespace msceqf::vision

#endif  // FEATURES_HPP