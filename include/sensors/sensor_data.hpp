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

#ifndef INPUT_HPP
#define INPUT_HPP

#include <opencv2/opencv.hpp>

#include "types/fptypes.hpp"
#include "vision/features.hpp"

namespace msceqf
{
/**
 * @brief Struct for one IMU reading.
 * It includes timestamp, angular velocity and linear acceleration.
 * -1 indicates an invalid timestamp.
 *
 */
struct Imu
{
  /**
   * @brief Get the IMU measurement as a 6 vector (ang, acc)
   *
   * @return R6 vector representing the IMU measurement as stacked angular velocity and linear acceleration
   */
  const Vector6 w() const { return (Vector6() << ang_, acc_).finished(); }

  /**
   * @brief get the IMU measurement as an extended matrix (ang, acc, 0)^ (SE23 lie algebra element)
   *
   * @return IMU measurement as a SE23 lie algebra element
   */
  const Matrix5 W() const
  {
    Matrix5 W = Matrix5::Zero();
    W.block<3, 3>(0, 0) = SO3::wedge(ang_);
    W.block<3, 1>(0, 3) = acc_;
    return W;
  }

  /**
   * @brief Comparison operator with other imu
   *
   */
  friend bool operator<(const Imu& lhs, const Imu& rhs) { return lhs.timestamp_ < rhs.timestamp_; }

  /**
   * @brief Comparison operator with timestamp
   *
   */
  friend bool operator<(const Imu& lhs, const fp& timestamp) { return lhs.timestamp_ < timestamp; }
  friend bool operator<(const fp& timestamp, const Imu& rhs) { return timestamp < rhs.timestamp_; }

  /**
   * @brief Stream an Imu
   *
   */
  friend std::ostream& operator<<(std::ostream& stream, Imu const& imu)
  {
    return stream << "(" << imu.timestamp_ << ", " << imu.ang_.transpose() << ", " << imu.acc_.transpose() << ")";
  }

  Vector3 ang_ = Vector3::Zero();  //!< Angular velocity vector
  Vector3 acc_ = Vector3::Zero();  //!< Acceleration vector
  fp timestamp_ = -1;              //!< Timestamp of the IMU reading
};

struct Camera
{
  /**
   * @brief Comparison operator with other camera
   *
   */
  friend bool operator<(const Camera& lhs, const Camera& rhs) { return lhs.timestamp_ < rhs.timestamp_; }

  /**
   * @brief Comparison operator with timestamp
   *
   */
  friend bool operator<(const Camera& lhs, const fp& timestamp) { return lhs.timestamp_ < timestamp; }
  friend bool operator<(const fp& timestamp, const Camera& rhs) { return timestamp < rhs.timestamp_; }

  cv::Mat image_;      //!< The image taken from the camera
  cv::Mat mask_;       //!< The mask for the given image, 255 in valid reagions, 0 in regions to be masked out
  fp timestamp_ = -1;  //!< Timestamp of the Camera reading
};

struct TriangulatedFeatures
{
  /**
   * @brief Comparison operator with other imu
   *
   */
  friend bool operator<(const TriangulatedFeatures& lhs, const TriangulatedFeatures& rhs)
  {
    return lhs.timestamp_ < rhs.timestamp_;
  }

  /**
   * @brief Comparison operator with timestamp
   *
   */
  friend bool operator<(const TriangulatedFeatures& lhs, const fp& timestamp) { return lhs.timestamp_ < timestamp; }
  friend bool operator<(const fp& timestamp, const TriangulatedFeatures& rhs) { return timestamp < rhs.timestamp_; }

  Features features_;            //!< The features detected in the image
  std::vector<Vector3> points_;  //!< The 3D points corresponding to the features
  fp timestamp_ = -1;            //!< Timestamp of the Camera reading
};

}  // namespace msceqf

#endif  // INPUT_HPP