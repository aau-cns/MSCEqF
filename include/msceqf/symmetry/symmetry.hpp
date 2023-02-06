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

#ifndef SYMMETRY_HPP
#define SYMMETRY_HPP

#include "msceqf/state/state.hpp"
#include "sensors/sensor_data.hpp"
#include "msceqf/system/system.hpp"
#include "utils/tools.hpp"

namespace msceqf
{

class Symmetry
{
 public:
  /**
   * @brief Implement the right group action phi of the symmetry group, acting on the homogenous space (phi(X, xi))
   *
   * @param X a MSCEqFState object
   * @param xi a SystemState object
   * @return const SystemState
   */
  [[nodiscard]] static const SystemState phi(const MSCEqFState& X, const SystemState& xi);

  /**
   * @brief Implement the lift function. Lift the actual dynamics onto the symmetry group
   *
   * @param xi a SystemState object
   * @param u a Input (Imu) object
   * @return const SystemState::SystemStateAlgebraMap
   */
  [[nodiscard]] static const SystemState::SystemStateAlgebraMap lift(const SystemState& xi, const Imu& u);

  static const Matrix5 D;  //!< The D matrix

 private:
  Symmetry() = default;
};

}  // namespace msceqf

#endif  // SYMMETRY_HPP