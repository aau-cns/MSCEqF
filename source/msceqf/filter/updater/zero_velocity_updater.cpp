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

#include "msceqf/filter/updater/zero_velocity_updater.hpp"
#include "msceqf/symmetry/symmetry.hpp"
#include "utils/logger.hpp"

namespace msceqf
{
ZeroVelocityUpdater::ZeroVelocityUpdater(const ZeroVelocityUpdaterOptions& opts, const Checker& checker)
    : opts_(opts), checker_(checker), motion_(false)
{
}

void ZeroVelocityUpdater::setMotion() { motion_ = true; }

void ZeroVelocityUpdater::setMeasurement(const SE23& y) { y_ = y; }

bool ZeroVelocityUpdater::isActive(const Tracks& tracks)
{
  if (opts_.zero_velocity_update_ == ZeroVelocityUpdate::DISABLE)
  {
    return false;
  }

  if (opts_.zero_velocity_update_ == ZeroVelocityUpdate::BEGINNING && motion_)
  {
    return false;
  }

  motion_ = checker_.disparityCheck(tracks);

  return !motion_;
}

bool ZeroVelocityUpdater::zvUpdate(MSCEqFState& X, const SystemState& xi0) const
{
  Vector9 delta = SE23::log(xi0.T().inv() * y_ * X.D().inv());

  Matrix9 Sigma = X.subCov({MSCEqFStateElementName::Dd}).block(0, 0, 9, 9);

  Matrix9 R = Matrix9::Identity() * 0.05 * 0.05;
  R.block(0, 0, 3, 3) = Sigma.block(0, 0, 3, 3);
  R.block(6, 6, 3, 3) = Sigma.block(6, 6, 3, 3);

  MatrixX G = X.subCovCols({MSCEqFStateElementName::Dd}).leftCols(9);
  MatrixX S(R.rows(), R.cols());
  S.triangularView<Eigen::Upper>() = Sigma + R;
  MatrixX invS = MatrixX::Identity(R.rows(), R.cols());
  S.selfadjointView<Eigen::Upper>().ldlt().solveInPlace(invS);
  MatrixX K = G * invS.selfadjointView<Eigen::Upper>();
  VectorX inn = K * delta;

  // Update state
  X.state_.at(MSCEqFStateElementName::Dd)
      ->updateLeft(inn.segment(X.index(MSCEqFStateElementName::Dd), X.dof(MSCEqFStateElementName::Dd)));
  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    X.state_.at(MSCEqFStateElementName::E)
        ->updateLeft(inn.segment(X.index(MSCEqFStateElementName::E), X.dof(MSCEqFStateElementName::E)));
  }
  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    X.state_.at(MSCEqFStateElementName::L)
        ->updateLeft(inn.segment(X.index(MSCEqFStateElementName::L), X.dof(MSCEqFStateElementName::L)));
  }
  for (auto& [timestamp, clone] : X.clones_)
  {
    clone->updateLeft(inn.segment(clone->getIndex(), clone->getDof()));
  }

  X.cov_.triangularView<Eigen::Upper>() -= K * G.transpose();
  X.cov_ = X.cov_.selfadjointView<Eigen::Upper>();

  if (opts_.curvature_correction_)
  {
    MatrixX expGamma = Symmetry::curvatureCorrection(X, inn);
    X.cov_ = expGamma * X.cov_ * expGamma.transpose();
    // I could use multleft mulright
  }

  return true;
}

}  // namespace msceqf