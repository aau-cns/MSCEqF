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

#ifndef FEATURE_HPP
#define FEATURE_HPP

#include <opencv2/opencv.hpp>

#include "types/fptypes.hpp"

namespace msceqf
{

/**
 * @brief feature class. Define a single feature.
 *
 * @note Note that feature cordinates are of cv::Point2f type for compatibility with OpenCV.
 *
 */
struct Feature
{
  using FeatureCoordinates = cv::Point2f;  //!< The feature coordinates

  /**
   * @brief Feature constructor
   *
   * @param uv
   * @param normalized_uv
   * @param timestamp
   * @param id
   */
  Feature(const FeatureCoordinates& uv, const FeatureCoordinates& normalized_uv, const fp& timestamp, const uint& id)
      : uv_(uv), normalized_uv_(normalized_uv), timestamp_(timestamp), id_(id)
  {
  }

  /**
   * @brief Feature constructor
   *
   * @param u
   * @param v
   * @param normalized_u
   * @param normalized_v
   * @param timestamp
   * @param id
   */
  Feature(const float& u,
          const float& v,
          const float& normalized_u,
          const float& normalized_v,
          const fp& timestamp,
          const uint& id)
      : uv_(u, v), normalized_uv_(normalized_u, normalized_v), timestamp_(timestamp), id_(id)
  {
  }

  /**
   * @brief Comparison operator with other feature
   *
   */
  friend bool operator<(const Feature& lhs, const Feature& rhs) { return lhs.timestamp_ < rhs.timestamp_; }

  /**
   * @brief Comparison operator with given timestamp
   *
   */
  friend bool operator<(const Feature& lhs, const fp& timestamp) { return lhs.timestamp_ < timestamp; }
  friend bool operator<(const fp& timestamp, const Feature& rhs) { return timestamp < rhs.timestamp_; }

  FeatureCoordinates uv_;             //!< (u, v) coordinates of the feature
  FeatureCoordinates normalized_uv_;  //!< Normalized (u, v) coordinates of the feature
  fp timestamp_;                      //!< Timestamp of the camera measurement containing the feature
  uint id_;                           //!< Id of the feature
};

}  // namespace msceqf

#endif  // FEATURE_HPP