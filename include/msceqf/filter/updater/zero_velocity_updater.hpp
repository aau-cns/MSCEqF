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

#ifndef ZERO_VELOCITY_UPDATER_HPP
#define ZERO_VELOCITY_UPDATER_HPP

#include "msceqf/filter/checker/checker.hpp"
#include "msceqf/state/state.hpp"

namespace msceqf
{
/**
 * @brief Zero velocity updater class. This class implements the Equivariant Zero Velocity Update (ZVU) of the MSCEqF
 * filter.
 *
 */
class ZeroVelocityUpdater
{
 public:
  /**
   * @brief Zero velocity updater constructor.
   *
   * @param opts Zero velocity updater options
   * @param checker Refernece to the MSCEqF checker
   */
  ZeroVelocityUpdater(const ZeroVelocityUpdaterOptions& opts, const Checker& checker);

  /**
   * @brief Set motion flag indicating that we are not in the static phase at the beginning but we have moved.
   *
   */
  void setMotion();

  /**
   * @brief Set the static extended pose measurement
   *
   * @param y static extended pose
   */
  void setMeasurement(const SE23& y);

  /**
   * @brief Check whether the zero velocity updater is active
   *
   * @param tracks tracks up to date used for disparity check
   *
   * @return true if the zero velocity updater is active, and therefore the platform is in static phase, false
   * otherwise
   */
  [[nodiscard]] bool isActive(const Tracks& tracks);

  /**
   * @brief Perform a zero velocity update
   *
   * @param X MSCEqF state
   * @param xi0 Origin state
   *
   * @return always true, when zero velocity update has been performed
   */
  [[nodiscard]] bool zvUpdate(MSCEqFState& X, const SystemState& xi0) const;

 private:
  ZeroVelocityUpdaterOptions opts_;  //!< The zero velocity updater options

  const Checker& checker_;  // The MSCEqF checker

  SE23 y_;  //!< Static extended pose

  bool motion_;  //!< Flag indicating whether we have moved
};

}  // namespace msceqf

#endif  // ZERO_VELOCITY_UPDATER_HPP