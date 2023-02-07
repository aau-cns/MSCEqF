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

#ifndef UPDATER_HPP
#define UPDATER_HPP

#include <unordered_set>

#include "msceqf/options/msceqf_options.hpp"
#include "msceqf/system/system.hpp"
#include "types/fptypes.hpp"
#include "vision/track.hpp"

namespace msceqf
{

class Updater
{
 public:
  Updater(const UpdaterOptions& opts);

  void update(MSCEqFState& X, const SystemState& xi0, const Tracks& tracks, const std::unordered_set<uint>& ids) const;

  /**
   * @brief Linear feature triangulation (DLT). This triangulates the given features using all the views the features is
   * seen from.
   *
   * @param X MSCEqF state
   * @param track track of the feature to triangulate
   * @param A_E Anchor E element
   * @param A_f triangulated feature
   * @return true if the triangulation was succesful, false otherwise
   */
  [[nodiscard]] bool linearTriangulation(const MSCEqFState& X, const Track& track, const SE3& A_E, Vector3& A_f) const;

  /**
   * @brief Nonlinear feature triangulation based on gauss newton. The feature is parametrized as anchored inverse depth
   * to render the optimization problem more stable.
   *
   * The optimization problem is build on the cost defined as the sum of the squared distances between the projections
   * of points on the unit plane.
   *
   *
   * @param X MSCEqF state
   * @param track track of the feature to triangulate
   * @param A_E Anchor E element
   * @param A_f triangulated feature
   */
  void nonlinearTriangulation(const MSCEqFState& X, const Track& track, const SE3& A_E, Vector3& A_f) const;

 private:
  /**
   * @brief Compute the residual for the nonlinear optimization
   *
   * @param X MSCEqF state
   * @param track track of the feature to triangulate
   * @param A_E Anchor E element
   * @param A_f Pre-triangulated feature
   * @param A_f_invdepth Inverse depth representation of pre-triangulated feature
   * @param res residual
   * @param J Jacobian
   */
  void nonlinearTriangulationResidualJacobian(const MSCEqFState& X,
                                              const Track& track,
                                              const SE3& A_E,
                                              const Vector3& A_f,
                                              const Vector3& A_f_invdepth,
                                              VectorX& res,
                                              MatrixX& J) const;

 private:
  UpdaterOptions opts_;  //!< The MSCEqF updater options
};

}  // namespace msceqf

#endif  // UPDATER_HPP