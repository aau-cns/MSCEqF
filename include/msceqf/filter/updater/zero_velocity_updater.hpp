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