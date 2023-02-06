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

#include "msceqf/options/msceqf_options.hpp"
#include "sensors/sensor_data.hpp"
#include "types/fptypes.hpp"
#include "utils/tools.hpp"
#include "vision/track.hpp"

namespace msceqf
{

class StaticInitializer
{
 public:
  using ImuBuffer = std::deque<Imu>;  //!< The Imu measurement buffer

  /**
   * @brief StaticInitializer constructor
   *
   * @param opts
   */
  StaticInitializer(const InitializerOptions& opts);

  /**
   * @brief Populate imu internal buffer used for acceleration check
   *
   * @param imu actual imu measurement
   */
  void insertImu(const Imu& imu);

  /**
   * @brief This function detects if the platform is moving based on acceleration measurements and image disparity
   *
   * @param tracks tracks up to date used for disparity check
   * @return true if motion is detected, flase otherwise
   */

  [[nodiscard]] bool detectMotion(const Tracks& tracks) const;

 private:
  /**
   * @brief This function returns true if the standard deviation of the collected acceleration measurements exceeds the
   * defined threshold.
   *
   * @return true if acceleration spike has been detected, false otherwise
   */
  [[nodiscard]] bool accelerationCheck() const;

  /**
   * @brief Perform disparity check
   *
   * @param tracks tracks up to date used for disparity check
   * @return true if disparity check succeed (diparity above threshold), false if no disparity is detected (disparity
   * below threshold)
   *
   * @note This method checks only tracks that are as long as the first track. This ideally should avoid to use newly
   * detected/tracked features corresponding to temporary objects moving in front of the camera
   */
  [[nodiscard]] bool disparityCheck(const Tracks& tracks) const;

  InitializerOptions opts_;  //!< The initializer options

  ImuBuffer imu_buffer_;  //!< The imu buffer used to check for acceleration spikes
};

}  // namespace msceqf

#endif  // STATIC_INITIALIZER_HPP