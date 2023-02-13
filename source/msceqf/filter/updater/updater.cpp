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
  if (ids.empty())
  {
    utils::Logger::warn("No tracks to update with");
    return;
  }

  // [TODO] For now let's use cov_.cols() and make it simple... Will work only with involved variables in a second stage

  // // Compute columns of C matrix and index map for each variable involved
  // std::unordered_map<std::variant<MSCEqFState::MSCEqFStateKey, fp>, size_t> index_map;
  // size_t index = 0;
  // if (X.opts_.enable_camera_extrinsics_calibration_)
  // {
  //   for (const auto& [timestamp, clone] : X.clones_)
  //   {
  //     index_map[timestamp] = index;
  //     index += clone->getDof();
  //   }
  // }
  // if (X.opts_.enable_camera_intrinsics_calibration_)
  // {
  //   index_map[MSCEqFStateElementName::L] = index;
  //   index += X.getPtr(MSCEqFStateElementName::L)->getDof();
  // }

  // // [TODO] Case of no extrinsic calibration, what variables are involved in the C matrix?

  // Compute the maximum number of rows of C matrix and residual
  size_t rows = 0;
  for (const auto& [id, track] : tracks)
  {
    rows += track.size();
  }
  rows *= ph_->block_rows_;

  // Preallocate C matrix and residual delta
  MatrixX C = MatrixX::Zero(rows, X.cov_.cols());
  VectorX delta = VectorX::Zero(rows);

  // Reset number of features in update
  total_size_ = 0;

  // For each track triangulate the feature, compute C and delta blocks, and performe chi2 rejection test
  for (const auto& id : ids)
  {
    const auto& track = tracks.at(id);

    if (track.size() < 2)
    {
      utils::Logger::debug("Track with id: " + std::to_string(id) + " do not contain enough views for triangulation");
      continue;
    }

    Vector3 A_f = Vector3::Zero();
    const auto& anchor = X.clones_.at(tracks.at(id).timestamps_.front());

    if (!linearTriangulation(X, track, anchor->E_, A_f))
    {
      utils::Logger::debug("Linear triangulation failed for track id: " + std::to_string(id));
      continue;
    }

    if (opts_.refine_traingulation_)
    {
      nonlinearTriangulation(X, track, anchor->E_, A_f);
    }

    const auto& track_size = track.size();

    // For each feature measurement in track compute the innovation block
    // (C matrix block, Cf matrix block and delta block)
    MatrixX Cf = MatrixX::Zero(ph_->block_rows_ * track_size, 3);
    for (size_t i = 0; i < track_size; ++i)
    {
      Vector2 uv(track.uvs_[i].x, track.uvs_[i].y);
      Vector2 uvn(track.normalized_uvs_[i].x, track.normalized_uvs_[i].y);
      FeatHelper feat(A_f, uv, uvn, anchor, X.clones_.at(track.timestamps_[i]), track.timestamps_[i]);

      const auto& row_idx = total_size_ + (ph_->block_rows_ * i);
      const auto& C_block = C.middleRows(row_idx, ph_->block_rows_);
      const auto& delta_block = delta.middleRows(row_idx, ph_->block_rows_);
      const auto& Cf_block = Cf.middleRows(ph_->block_rows_ * i, ph_->block_rows_);

      ph_->innovationBlock(X, xi0_, feat, C_block, delta_block, Cf_block);
    }

    auto C_block = C.middleRows(total_size_, ph_->block_rows_ * track_size);
    auto delta_block = delta.middleRows(total_size_, ph_->block_rows_ * track_size);

    // [DEBUG]
    // std::cout << "delta_block:" << delta_block.transpose() << std::endl;

    // Perform nullspace projection of Cf
    UpdaterHelper::nullspaceProjection(Cf, C_block, delta_block);

    // Perform chi2 test
    if (!UpdaterHelper::chi2Test(X, C_block, delta_block, opts_.pixel_std_, chi2_table_))
    {
      utils::Logger::debug("Chi2 test failed for track id: " + std::to_string(id));
      continue;
    }

    // Update total size of C matrix and residual delta
    total_size_ += ph_->block_rows_ * track_size;
  }

  if (total_size_ == 0)
  {
    utils::Logger::warn("No valid features to update with. Skipping update step");
    return;
  }

  // Resize residual delta and C matrix based on total_size_
  delta.conservativeResize(total_size_);
  C.conservativeResize(total_size_, C.cols());

  // Update compression
  if (C.rows() > C.cols())
  {
    UpdaterHelper::updateQRCompression(C, delta);
  }

  // Define measurement noise covariance
  MatrixX R = MatrixX::Identity(C.rows(), C.rows()) * opts_.pixel_std_ * opts_.pixel_std_;

  // MSCEqF Update
  UpdateMSCEqF(X, C, delta, R);
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
  Vector3 A_f_init = A_f;
  Vector3 A_f_invdepth(A_f(0) / A_f(2), A_f(1) / A_f(2), 1 / A_f(2));

  MatrixX J = MatrixX::Zero(2 * track.size(), 3);
  VectorX res = VectorX::Zero(2 * track.size());

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

    A_f(0) = A_f_invdepth(0) / A_f_invdepth(2);
    A_f(1) = A_f_invdepth(1) / A_f_invdepth(2);
    A_f(2) = 1.0 / A_f_invdepth(2);

    if (delta.norm() < opts_.tollerance_)
    {
      utils::Logger::debug("Feature refinement converged in " + std::to_string(iterations) + " iterations");
      converged = true;
      break;
    }
  }

  // Return given initial value if no improvement
  if (!converged)
  {
    utils::Logger::debug("Feature refinement reached max iterations");

    if (actual_res_norm > initial_residual_norm)
    {
      A_f = A_f_init;
      return;
    }
  }

  // Return given initial value if invalid
  if (A_f(2) < opts_.min_depth_ || A_f(2) > opts_.max_depth_ || std::isnan(A_f.norm()))
  {
    A_f = A_f_init;
    return;
  }
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

void Updater::UpdateMSCEqF(MSCEqFState& X, const MatrixX& C, const VectorX& delta, const MatrixX& R) const
{
  // [TODO] For now let's use cov_.cols() and make it simple... Will work only with involved variables in a second stage

  // // get covariance of variables involved in update
  // std::vector<MSCEqFState::MSCEqFStateKey> keys;
  // if (X.opts_.enable_camera_extrinsics_calibration_)
  // {
  //   keys.push_back(MSCEqFStateElementName::E);
  // }
  // else
  // {
  //   // [TODO]
  // }
  // if (X.opts_.enable_camera_intrinsics_calibration_)
  // {
  //   keys.push_back(MSCEqFStateElementName::L);
  // }

  // Compute Kalman gain and innovation
  MatrixX G = X.cov_ * C.transpose();
  // MatrixX S = C * X.subCov(keys) * C.transpose() + R;
  MatrixX S = C * X.cov() * C.transpose() + R;
  MatrixX K = G * S.inverse();
  VectorX inn = K * delta;

  // [DEBUG]
  std::cout << "delta: " << delta.transpose() << std::endl;
  std::cout << "inn: " << inn.transpose() << std::endl;

  // Update state
  X.state_.at(MSCEqFStateElementName::Dd)
      ->updateLeft(
          inn.segment(X.stateElementIndex(MSCEqFStateElementName::Dd), X.stateElementDof(MSCEqFStateElementName::Dd)));
  if (X.opts_.enable_camera_extrinsics_calibration_)
  {
    X.state_.at(MSCEqFStateElementName::E)
        ->updateLeft(
            inn.segment(X.stateElementIndex(MSCEqFStateElementName::E), X.stateElementDof(MSCEqFStateElementName::E)));
  }
  if (X.opts_.enable_camera_intrinsics_calibration_)
  {
    X.state_.at(MSCEqFStateElementName::L)
        ->updateLeft(
            inn.segment(X.stateElementIndex(MSCEqFStateElementName::L), X.stateElementDof(MSCEqFStateElementName::L)));
  }
  for (auto& [timestamp, clone] : X.clones_)
  {
    clone->updateLeft(inn.segment(clone->getIndex(), clone->getDof()));
  }

  // Update covariance
  X.cov_ -= K * G.transpose();
}

}  // namespace msceqf