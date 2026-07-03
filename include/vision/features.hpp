// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#ifndef FEATURES_HPP
#define FEATURES_HPP

#include <opencv2/opencv.hpp>

#include "types/fptypes.hpp"

namespace msceqf
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
   * @return Number of features
   */
  inline size_t size() const noexcept
  {
    assert(distorted_uvs_.size() == uvs_.size());
    assert(distorted_uvs_.size() == normalized_uvs_.size());
    assert(distorted_uvs_.size() == ids_.size());
    return distorted_uvs_.size();
  }

  /**
   * @brief Remove invalid features coordinates, normalized feature coordinates and ids given a vector of boolean flags
   * indicating invalid features
   *
   * @param invalid Vector of boolean flags indicating invalid features
   */
  void removeInvalid(std::vector<bool>& invalid)
  {
    assert(invalid.size() == distorted_uvs_.size());
    assert(distorted_uvs_.size() == uvs_.size());
    assert(distorted_uvs_.size() == normalized_uvs_.size());
    assert(distorted_uvs_.size() == ids_.size());

    size_t i = 0;
    size_t j = 0;

    while (i < invalid.size())
    {
      if (!invalid[i])
      {
        distorted_uvs_[j] = distorted_uvs_[i];
        uvs_[j] = uvs_[i];
        normalized_uvs_[j] = normalized_uvs_[i];
        ids_[j] = ids_[i];
        ++j;
      }
      ++i;
    }

    distorted_uvs_.resize(j);
    uvs_.resize(j);
    normalized_uvs_.resize(j);
    ids_.resize(j);
  }

  FeaturesCoordinates distorted_uvs_;   //!< Distorted (u, v) coordinates of the features detected/tracked
  FeaturesCoordinates uvs_;             //!< Undistorted (u, v) coordinates of the features detected/tracked
  FeaturesCoordinates normalized_uvs_;  //!< Undistorted normalized (u, v) coordinates of features detected/tracked
  FeatureIds ids_;                      //!< Id of the features detected/tracked
};

}  // namespace msceqf

#endif  // FEATURES_HPP