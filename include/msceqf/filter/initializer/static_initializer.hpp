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

#ifndef STATIC_INITIALIZER_HPP
#define STATIC_INITIALIZER_HPP

#include "msceqf/filter/checker/checker.hpp"
#include "sensors/sensor_data.hpp"

namespace msceqf
{
class StaticInitializer
{
 public:
  using ImuBuffer = std::deque<Imu>;  //!< The Imu measurement buffer

  /**
   * @brief StaticInitializer constructor
   *
   * @param opts Initializer options
   * @param checker Refernece to the MSCEqF checker
   */
  StaticInitializer(const InitializerOptions& opts, const Checker& checker);

  /**
   * @brief Populate imu internal buffer used for acceleration check
   *
   * @param imu Actual imu measurement
   */
  void insertImu(const Imu& imu);

  /**
   * @brief This function detects if the platform is moving based on acceleration measurements and image disparity
   *
   * @param tracks Tracks up to date used for disparity check
   * @return true if motion is detected, flase otherwise
   */

  [[nodiscard]] bool detectMotion(const Tracks& tracks);

  /**
   * @brief This fnctions collects a predefined window of IMU measurments and compute the roll and pitch fo the platform
   * togheter with the IMU bias without waiting for motion
   *
   * @return true if the initialization of the origin has succeedded, false otherwise
   */
  [[nodiscard]] bool initializeOrigin();

  /**
   * @brief This function returns the initial Extended pose of the platform, to be used as origin
   *
   * @return Initial extended pose of the platform (orientation, velocity and position)
   */
  [[nodiscard]] const SE23& T0() const;

  /**
   * @brief This function returns the initial IMU bias, to be used as origin
   *
   * @return Initial IMU bias (gyroscope bias and accelerometer bias)
   */
  [[nodiscard]] const Vector6& b0() const;

 private:
  /**
   * @brief This function is used to detect an acceleration spike, corresponding to a state transition from static
   * condition to motion. The function returns true if the standard deviation of the collected acceleration measurements
   * exceeds the defined threshold. If motion is detected, the initial extended pose and bias are set
   *
   * @return true if acceleration spike has been detected, false otherwise
   */
  [[nodiscard]] bool detectAccelerationSpike();

  /**
   * @brief This function computes the mean acceleration and angular velocity of the IMU measurements in the IMU buffer,
   * as well as the standard deviation of the acceleration measurements
   *
   * @return true if the buffer contains enough measurements and the computation of the means and standard deviation has
   * succeedded, false otherwise
   *
   * @param acc_mean Mean of acceleration measurements
   * @param ang_mean Mean of angular velocity measurements
   * @param acc_std Standard deviation of the acceleration measurements
   */
  [[nodiscard]] bool imuMeanStd(Vector3& acc_mean, Vector3& ang_mean, fp& acc_std) const;

  /**
   * @brief This function computes and set the origin of the platform based on the mean acceleration and angular
   * velocity
   *
   * @param acc_mean Mean of acceleration measurements
   * @param ang_mean Mean of angular velocity measurements
   */
  void computeOrigin(Vector3& acc_mean, Vector3& ang_mean);

  InitializerOptions opts_;  //!< The initializer options

  const Checker& checker_;  // The MSCEqF checker

  ImuBuffer imu_buffer_;  //!< The imu buffer used to check for acceleration spikes

  SE23 T0_;     //!< The initial extended pose of the platform
  Vector6 b0_;  //!< The initial IMU bias
};

}  // namespace msceqf

#endif  // STATIC_INITIALIZER_HPP