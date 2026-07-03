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

#ifndef SYMMETRY_HPP
#define SYMMETRY_HPP

#include "msceqf/state/state.hpp"
#include "sensors/sensor_data.hpp"
#include "msceqf/state/state.hpp"
#include "utils/tools.hpp"

namespace msceqf
{
class Symmetry
{
 public:
  /**
   * @brief Implement the right group action phi of the symmetry group, acting on the homogenous space (phi(X, xi))
   *
   * @param X MSCEqF state (symmetry group element)
   * @param xi System state (homogenous space element)
   * @return System state (homogenous space element)
   */
  [[nodiscard]] static const SystemState phi(const MSCEqFState& X, const SystemState& xi);

  /**
   * @brief Implement the lift function. Lift the actual dynamics onto the symmetry group
   *
   * @param xi System state (homogenous space element)
   * @param u Input (Imu)
   * @return Input for the lifted system (Symmetry group Lie algebra element)
   */
  [[nodiscard]] static const SystemState::SystemStateAlgebraMap lift(const SystemState& xi, const Imu& u);

  /**
   * @brief Return the Gamma matrix for the reset / curvature correction
   *
   * @param X MSCEqF state (symmetry group element)
   * @param inn Innovatiation vector
   * @return Gamma matrix
   */
  [[nodiscard]] static const MatrixX curvatureCorrection(const MSCEqFState& X, const VectorX& inn);

  static const Matrix5 D;  //!< The D matrix

 private:
  Symmetry() = default;
};

}  // namespace msceqf

#endif  // SYMMETRY_HPP