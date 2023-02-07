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

#include "msceqf/filter/updater/updater.hpp"

#include <opencv2/opencv.hpp>

#include "utils/logger.hpp"

namespace msceqf
{

Updater::Updater(const UpdaterOptions& opts) : opts_(opts) {}

void Updater::update([[maybe_unused]] MSCEqFState& X,
                     [[maybe_unused]] const SystemState& xi0,
                     const Tracks& tracks,
                     const std::unordered_set<uint>& ids) const
{
  for (const auto& id : ids)
  {
    if (tracks.at(id).size() < 2)
    {
      continue;
    }

    Vector3 A_f = Vector3::Zero();
    const SE3& A_E = X.clones_.at(tracks.at(id).timestamps_.front())->E_;

    if (!linearTriangulation(X, tracks.at(id), A_E, A_f))
    {
      continue;
    }

    if (opts_.refine_traingulation_)
    {
      nonlinearTriangulation(X, tracks.at(id), A_E, A_f);
    }
  }
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
    nonlinearTriangulationResidualJacobian(X, track, A_E, A_f, A_f_invdepth, res, J);
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

void Updater::nonlinearTriangulationResidualJacobian(const MSCEqFState& X,
                                                     const Track& track,
                                                     const SE3& A_E,
                                                     const Vector3& A_f,
                                                     const Vector3& A_f_invdepth,
                                                     VectorX& res,
                                                     MatrixX& J) const
{
  J.setZero();
  res.setZero();

  Eigen::Matrix<fp, 3, 2> J_rep = Eigen::Matrix<fp, 3, 2>::Zero();

  J_rep.block<2, 2>(0, 0) = Matrix2::Identity();
  J_rep.block<3, 1>(0, 2) = -A_f_invdepth;
  J_rep = A_f_invdepth(2) * J_rep;

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