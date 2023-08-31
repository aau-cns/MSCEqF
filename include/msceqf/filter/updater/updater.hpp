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

#include "types/fptypes.hpp"
#include "msceqf/options/msceqf_options.hpp"
#include "msceqf/filter/updater/updater_helper.hpp"
#include "msceqf/system/system.hpp"
#include "vision/track.hpp"

namespace msceqf
{
/**
 * @brief Updater class. This class implements the Multi State Constraint update step of the MSCEqF filter.
 *
 */
class Updater
{
 public:
  Updater(const UpdaterOptions& opts, const SystemState& xi0);

  /**
   * @brief Perform a Multi State Constraint update
   *
   * @param X MSCEqF state
   * @param tracks Tracks to update
   * @param ids Indices of the tracks that are evaluated for an update
   *
   * @note Not all the tracks corresponding to the given ids will be used for the update. Tracks that do not contains at
   * least two views, tracks for which the triangulation fails, and tracks that fail the chi2 test are discarded. The
   * set ids will change accordingly.
   */
  void mscUpdate(MSCEqFState& X, const Tracks& tracks, std::unordered_set<uint>& ids);

 private:
  /**
   * @brief Linear feature triangulation (DLT). This triangulates the given features using all the views the features is
   * seen from.
   *
   * @param X MSCEqF state
   * @param track Track of the feature to triangulate
   * @param A_E Anchor E element
   * @param A_f Triangulated feature
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
   * @param track Track of the feature to triangulate
   * @param A_E Anchor E element
   * @param A_f Triangulated feature
   */
  void nonlinearTriangulation(const MSCEqFState& X, const Track& track, const SE3& A_E, Vector3& A_f) const;

  /**
   * @brief Compute the residual for the nonlinear optimization
   *
   * @param X MSCEqF state
   * @param track Track of the feature to triangulate
   * @param A_E Anchor E element
   * @param A_f Pre-triangulated feature
   * @param res Residual
   * @param J Jacobian
   */
  void nonlinearTriangulationResidualJacobian(
      const MSCEqFState& X, const Track& track, const SE3& A_E, const Vector3& A_f, VectorX& res, MatrixX& J) const;

  /**
   * @brief Perfom the update step of the MSCEqF filter
   *
   * @param X MSCEqF state
   * @param C Ouput matrix
   * @param delta Residual delta
   * @param R Measurement noise covariance
   */
  void UpdateMSCEqF(MSCEqFState& X, const MatrixX& C, const VectorX& delta, const MatrixX& R) const;

 private:
  UpdaterOptions opts_;  //!< The MSCEqF updater options

  const SystemState& xi0_;  //!< The system state origin

  ProjectionHelperUniquePtr ph_;  //!< Projection helper

  std::map<uint, fp> chi2_table_;  //!< Chi squared table for outlier rejection

  ColsMap cols_map_;  //!< Map of the columns of the C matrix and residual delta

  std::vector<uint> update_ids_;  //!< Ids of the tracks used in the update
  size_t total_size_;             //!< Total size of C matrix and residual for update
};

}  // namespace msceqf

#endif  // UPDATER_HPP