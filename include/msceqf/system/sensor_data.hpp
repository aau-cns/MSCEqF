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

#ifndef INPUT_HPP
#define INPUT_HPP

#include "types/fptypes.hpp"

namespace msceqf
{
/**
 * @brief Struct for one IMU reading.
 * It includes timestamp, angular velocity and linear acceleration
 *
 */
struct Imu
{
  fp timestamp_ = 0;             //!< Timestamp of the IMU reading
  Vector3 w_ = Vector3::Zero();  //!< Angular velocity vector
  Vector3 a_ = Vector3::Zero();  //!< Acceleration vector

  const Matrix5 W() const
  {
    Matrix5 W = Matrix5::Zero();
    W.block<3, 3>(0, 0) = SO3::wedge(w_);
    W.block<3, 1>(0, 3) = a_;
    return W;
  }

  /**
   * @brief Comparison operator
   *
   */
  bool operator<(const Imu& other) const { return timestamp_ < other.timestamp_; }
};

}  // namespace msceqf

#endif  // INPUT_HPP