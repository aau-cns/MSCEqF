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

#include <opencv2/opencv.hpp>

#include "msceqf/filter/updater/updater.hpp"
#include "utils/logger.hpp"

namespace msceqf
{

Updater::Updater(const UpdaterOptions& opts, const SystemState& xi0)
    : opts_(opts), xi0_(xi0), ph_(nullptr), chi2_table_(), total_size_(0)
{
  switch (opts_.projection_method_)
  {
    case ProjectionMethod::UNIT_SPHERE:
      ph_ = createProjectionHelper<ProjectionHelperS2>(opts_.msc_features_representation_);
      break;
    case ProjectionMethod::UNIT_PLANE:
      ph_ = createProjectionHelper<ProjectionHelperZ1>(opts_.msc_features_representation_);
      break;
    default:
      break;
  }

  for (uint dof = 1; dof <= 1000; ++dof)
  {
    boost::math::chi_squared chi_squared_dist(dof);
    chi2_table_[dof] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void Updater::update(MSCEqFState& X, const Tracks& tracks, const std::unordered_set<uint>& ids)
{
  // Compute number of columns of C matrix
  size_t cols = 15;
  if (X.opts_.enable_camera_extrinsics_calibration_)
  {
    cols += 6;
  }
  if (X.opts_.enable_camera_intrinsics_calibration_)
  {
    cols += 4;
  }

  // Compute the maximum number of rows of C matrix and residual
  size_t rows = 0;
  for (const auto& [id, track] : tracks)
  {
    rows += track.size();
  }
  rows *= ph_->block_rows_;

  // Preallocate C matrix and residual
  MatrixX C = MatrixX::Zero(rows, cols);
  VectorX res = VectorX::Zero(rows);

  // Reset number of features in update
  total_size_ = 0;

  // For each track triangulate the feature, compute C matrices and residual blocks, and performe chi2 rejection test
  for (const auto& id : ids)
  {
    const auto& track = tracks.at(id);

    if (track.size() < 2)
    {
      continue;
    }

    Vector3 A_f = Vector3::Zero();
    const SE3& A_E = X.clones_.at(tracks.at(id).timestamps_.front())->E_;

    if (!linearTriangulation(X, track, A_E, A_f))
    {
      utils::Logger::debug("Linear triangulation failed for track id: " + std::to_string(id));
      continue;
    }

    if (opts_.refine_traingulation_)
    {
      nonlinearTriangulation(X, track, A_E, A_f);
    }

    const auto& track_size = track.size();

    // For each feature measurement in track compute the innovation block
    // (Ct matrix block, Cf matrix block and residual block)
    MatrixX Cf = MatrixX::Zero(ph_->block_rows_ * track_size, 3);
    for (size_t i = 0; i < track_size; ++i)
    {
      Vector2 uvn(track.normalized_uvs_[i].x, track.normalized_uvs_[i].y);
      FeatHelper feat(A_E, A_f, uvn, X.clones_.at(track.timestamps_[i]), track.timestamps_[i]);

      const auto& row_idx = total_size_ + (ph_->block_rows_ * i);

      ph_->innovationBlock(X, xi0_, feat, C.middleRows(row_idx, ph_->block_rows_),
                           res.middleRows(row_idx, ph_->block_rows_),
                           Cf.middleRows(ph_->block_rows_ * i, ph_->block_rows_));
    }

    const auto& Ct_block = C.middleRows(total_size_, ph_->block_rows_ * track_size);
    const auto& res_block = res.middleRows(total_size_, ph_->block_rows_ * track_size);

    // Perform nullspace projection of Cf
    UpdaterHelper::nullspaceProjection(Cf, Ct_block, res_block);

    // Perform chi2 test
    if (!UpdaterHelper::chi2Test(X, Ct_block, res_block, opts_.pixel_std_, chi2_table_))
    {
      utils::Logger::debug("Chi2 test failed for track id: " + std::to_string(id));
      continue;
    }

    // Update total size of C matrix and residual
    total_size_ += ph_->block_rows_ * track_size;
  }

  // Resize residual and C matrix based on total_size_
  res.conservativeResize(total_size_);
  C.conservativeResize(total_size_, cols);

  // [TODO] Update compression
  // [TODO] Update
}

bool Updater::linearTriangulation(const MSCEqFState& X, const Track& track, const SE3& A_E, Vector3& A_f) const
{
  Matrix3 A = Matrix3::Zero();
  Vector3 b = Vector3::Zero();

  Matrix3 Ai = Matrix3::Zero();
  Vector3 A_bf = Vector3::Zero();

  for (size_t i = 0; i < track.size(); ++i)
  {
    SE3 E = A_E.inv() * X.clones_.at(track.timestamps_[i])->E_;

    A_bf(0) = track.normalized_uvs_[i].x;
    A_bf(1) = track.normalized_uvs_[i].y;
    A_bf(2) = 1.0;

    A_bf = E.R() * A_bf;
    Ai = -SO3::wedge(A_bf) * SO3::wedge(A_bf);

    A += Ai;
    b += Ai * E.x();
  }

  A_f = A.colPivHouseholderQr().solve(b);

  if (A_f(2) < opts_.min_depth_ || A_f(2) > opts_.max_depth_ || std::isnan(A_f.norm()))
  {
    return false;
  }

  return true;
}

void Updater::nonlinearTriangulation(const MSCEqFState& X, const Track& track, const SE3& A_E, Vector3& A_f) const
{
  Vector3 A_f_invdepth(A_f(0) / A_f(2), A_f(1) / A_f(2), 1 / A_f(2));

  MatrixX J = MatrixX::Zero(2 * X.clones_.size(), 3);
  VectorX res = VectorX::Zero(2 * X.clones_.size());

  fp initial_residual_norm;
  fp actual_res_norm;

  bool converged = false;

  for (uint iterations = 0; iterations < opts_.max_iterations_; ++iterations)
  {
    nonlinearTriangulationResidualJacobian(X, track, A_E, A_f, res, J);
    Vector3 delta = J.colPivHouseholderQr().solve(res);

    actual_res_norm = res.norm();

    if (iterations == 0)
    {
      initial_residual_norm = actual_res_norm;
    }

    A_f_invdepth += delta;

    if (delta.norm() < opts_.tollerance_)
    {
      utils::Logger::debug("Feature refinement converged in " + std::to_string(iterations) + " iterations");
      converged = true;
      break;
    }
  }

  // Return if no improvement
  if (!converged)
  {
    utils::Logger::debug("Feature refinement reached max iterations");

    if (actual_res_norm > initial_residual_norm)
    {
      return;
    }
  }

  // Return if no improvement
  Vector3 A_f_tpm(A_f_invdepth(0) / A_f_invdepth(2), A_f_invdepth(1) / A_f_invdepth(2), 1.0 / A_f_invdepth(2));
  if (A_f_tpm(2) < opts_.min_depth_ || A_f_tpm(2) > opts_.max_depth_ || std::isnan(A_f_tpm.norm()))
  {
    return;
  }

  A_f = A_f_tpm;
}

void Updater::nonlinearTriangulationResidualJacobian(
    const MSCEqFState& X, const Track& track, const SE3& A_E, const Vector3& A_f, VectorX& res, MatrixX& J) const
{
  J.setZero();
  res.setZero();

  Matrix3 J_rep = Matrix3::Identity();
  J_rep.block<3, 1>(0, 2) = -A_f;
  J_rep = A_f(2) * J_rep;

  for (size_t i = 0; i < track.size(); ++i)
  {
    SE3 E = X.clones_.at(track.timestamps_[i])->E_.inv() * A_E;

    Vector3 Ci_f = E * A_f;
    Vector3 Ci_f_invdepth(Ci_f(0) / Ci_f(2), Ci_f(1) / Ci_f(2), 1 / Ci_f(2));

    Vector2 uvn(track.normalized_uvs_[i].x, track.normalized_uvs_[i].y);
    res.block(2 * i, 0, 2, 1) = uvn - Ci_f_invdepth.block(0, 0, 2, 1);

    J.block(2 * i, 0, 2, 2) = Matrix2::Identity();
    J.block(2 * i, 2, 2, 1) = -Ci_f_invdepth.block<2, 1>(0, 0);
    J.block(2 * i, 0, 2, 3) = Ci_f_invdepth(2) * J.block(2 * i, 0, 2, 3) * E.R() * J_rep;
  }
}

}  // namespace msceqf