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

#ifndef TRACK_HPP
#define TRACK_HPP

#include <opencv2/opencv.hpp>

#include "types/fptypes.hpp"
#include "vision/features.hpp"

namespace msceqf
{
/**
 * @brief (Cache friendly) Track struct. Define a feature (labeled via a feature id) detected/tracked at different
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
   * @return Amount of features
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
   * @param invalid Vector of boolean flags indicating invalid features
   */
  void removeInvalid(std::vector<bool>& invalid)
  {
    assert(invalid.size() == uvs_.size());
    assert(uvs_.size() == normalized_uvs_.size());
    assert(uvs_.size() == timestamps_.size());

    size_t i = 0;
    size_t j = 0;

    while (i < uvs_.size())
    {
      if (!invalid[i])
      {
        uvs_[j] = uvs_[i];
        normalized_uvs_[j] = normalized_uvs_[i];
        timestamps_[j] = timestamps_[i];
        ++j;
      }
      ++i;
    }

    uvs_.resize(j);
    normalized_uvs_.resize(j);
    timestamps_.resize(j);
  }

  /**
   * @brief Remove the tail of the track. this method removes coordinates and timestamps that are older or equal than
   * the given timestamp
   *
   * @param timestamp Timestamp
   * @param remove_equal Flag to indicate whether to include in the removal also the coordinates and timestamps at the
   * given timestamp
   */
  void removeTail(const fp& timestamp, const bool& remove_equal = true)
  {
    assert(uvs_.size() == normalized_uvs_.size());
    assert(uvs_.size() == timestamps_.size());

    size_t j = 0;
    size_t i = 0;

    while (i < uvs_.size())
    {
      if (remove_equal && timestamps_[i] > timestamp)
      {
        uvs_[j] = uvs_[i];
        normalized_uvs_[j] = normalized_uvs_[i];
        timestamps_[j] = timestamps_[i];
        ++j;
      }
      if (!remove_equal && timestamps_[i] >= timestamp)
      {
        uvs_[j] = uvs_[i];
        normalized_uvs_[j] = normalized_uvs_[i];
        timestamps_[j] = timestamps_[i];
        ++j;
      }
      ++i;
    }

    uvs_.resize(j);
    normalized_uvs_.resize(j);
    timestamps_.resize(j);
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

using Tracks = std::unordered_map<uint, Track>;  //!< Tracks defined as a a vector of tracks mapped by ids

}  // namespace msceqf

#endif  // TRACK_HPP