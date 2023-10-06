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

#include "msceqf/filter/updater/updater_helper.hpp"

namespace msceqf
{
ProjectionHelper::ProjectionHelper(const FeatureRepresentation& feature_representation)
    : feature_representation_(feature_representation)
{
  switch (feature_representation_)
  {
    case FeatureRepresentation::ANCHORED_EUCLIDEAN:
      dim_loss_ = 3;
      break;
    case FeatureRepresentation::ANCHORED_INVERSE_DEPTH:
      dim_loss_ = 3;
      break;
    case FeatureRepresentation::ANCHORED_POLAR:
      dim_loss_ = 4;
      break;
  }
}

Vector3 ProjectionHelperS2::pi(const Vector3& f) { return f / f.norm(); }

Vector3 ProjectionHelperZ1::pi(const Vector3& f) { return (Vector3() << f(0) / f(2), f(1) / f(2), 1.0).finished(); }

MatrixX ProjectionHelperS2::dpi(const Vector3& f)
{
  return (Matrix3::Identity() - ((f * f.transpose()) / (f.transpose() * f))) / f.norm();
}

MatrixX ProjectionHelperZ1::dpi(const Vector3& f)
{
  return (Eigen::Matrix<fp, 2, 3>() << 1.0 / f(2), 0.0, -f(0) / (f(2) * f(2)), 0.0, 1.0 / f(2), -f(1) / (f(2) * f(2)))
      .finished();
}

void ProjectionHelperS2::residualJacobianBlock([[maybe_unused]] const MSCEqFState& X,
                                               [[maybe_unused]] const SystemState& xi0,
                                               [[maybe_unused]] const FeatHelper& feat,
                                               [[maybe_unused]] MatrixXBlockRowRef C_block_row,
                                               [[maybe_unused]] VectorXBlockRowRef delta_block_row,
                                               [[maybe_unused]] MatrixXBlockRowRef Cf_block_row,
                                               [[maybe_unused]] const ColsMap& cols_map)
{
  throw std::runtime_error("Update with S2 projection not implemented yet");
}

void ProjectionHelperZ1::residualJacobianBlock(const MSCEqFState& X,
                                               const SystemState& xi0,
                                               const FeatHelper& feat,
                                               MatrixXBlockRowRef C_block_row,
                                               VectorXBlockRowRef delta_block_row,
                                               MatrixXBlockRowRef Cf_block_row,
                                               const ColsMap& cols_map)
{
  const auto& anchor_E = X.clone(feat.anchor_timestamp_);
  const auto& clone_E = X.clone(feat.clone_timestamp_);

  // feature in origin frame and camera frame
  Vector3 G0_f = anchor_E * feat.A_f_;
  Vector3 C_f = clone_E.inv() * G0_f;

  // precompute D = K0 * L * dpi(C_f) if intrinsics are calibrated, D = dpi(C_f) otherwise
  Eigen::Matrix<fp, 2, 3> D = X.opts().enable_camera_intrinsics_calibration_ ?
                                  (xi0.K() * X.L()).asMatrix().block<2, 2>(0, 0) * dpi(C_f) :
                                  dpi(C_f);

  // Precompute P = L * pi(C_f) if intrinsics are calibrated, P = pi(C_f) otherwise
  Vector3 P = X.opts().enable_camera_intrinsics_calibration_ ? X.L().asMatrix() * pi(C_f) : pi(C_f);

  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    C_block_row.block(0, cols_map.at(MSCEqFStateElementName::L), block_rows_, X.dof(MSCEqFStateElementName::L))
        .noalias() = xi0.K().asMatrix().block<2, 2>(0, 0) * UpdaterHelper::Xi(P);
  }

  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    Eigen::Matrix<fp, 3, 6> A = Eigen::Matrix<fp, 3, 6>::Zero();
    A.block<3, 3>(0, 0) = SO3::wedge(G0_f);
    A.block<3, 3>(0, 3) = -Matrix3::Identity();

    if (feat.clone_timestamp_ != feat.anchor_timestamp_)
    {
      C_block_row.block(0, cols_map.at(feat.clone_timestamp_), block_rows_, X.dof(feat.clone_timestamp_)).noalias() =
          D * clone_E.R().transpose() * A;
      C_block_row.block(0, cols_map.at(feat.anchor_timestamp_), block_rows_, X.dof(feat.anchor_timestamp_)) =
          -C_block_row.block(0, cols_map.at(feat.clone_timestamp_), block_rows_, X.dof(feat.clone_timestamp_));
    }
  }
  else
  {
    // [TODO] Ct if extrinsics are not calibrated
  }

  switch (feature_representation_)
  {
    case FeatureRepresentation::ANCHORED_EUCLIDEAN:
      Cf_block_row = D * clone_E.R().transpose() * anchor_E.R();
      break;
    case FeatureRepresentation::ANCHORED_INVERSE_DEPTH:
      Cf_block_row = D * clone_E.R().transpose() * anchor_E.R() * UpdaterHelper::inverseDepthJacobian(feat.A_f_);
      break;
    case FeatureRepresentation::ANCHORED_POLAR:
      Vector3 A_f0 = (Vector3() << 0.0, 0.0, 1.0).finished();
      Vector3 thetak =
          std::acos((A_f0.normalized().transpose() * feat.A_f_.normalized())) * A_f0.cross(feat.A_f_).normalized();
      Eigen::Matrix<fp, 3, 4> J = Eigen::Matrix<fp, 3, 4>::Zero();
      J.block<3, 3>(0, 0) = SO3::wedge(feat.A_f_) * SO3::leftJacobian(thetak);
      J.block<3, 1>(0, 3) = -feat.A_f_;
      Cf_block_row = D * clone_E.R().transpose() * anchor_E.R() * J;
      break;
  }

  // Residual
  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    delta_block_row = feat.uv_ - (xi0.K().asMatrix() * P).segment<2>(0);
  }
  else
  {
    delta_block_row = feat.uvn_ - P.segment<2>(0);
  }
}

Eigen::Matrix<fp, 2, 4> UpdaterHelper::Xi(const Vector3& f)
{
  Eigen::Matrix<fp, 2, 4> Xi = Eigen::Matrix<fp, 2, 4>::Zero();
  Xi(0, 0) = f(0);
  Xi(1, 1) = f(1);
  Xi(0, 2) = f(2);
  Xi(1, 3) = f(2);
  return Xi;
}

Matrix3 UpdaterHelper::inverseDepthJacobian(const Vector3& A_f)
{
  Matrix3 Cid = Matrix3::Identity();
  Cid.block<3, 1>(0, 2) = -A_f;
  Cid = A_f(2) * Cid;
  return Cid;
}

void UpdaterHelper::nullspaceProjection(Eigen::Ref<MatrixX> Cf, MatrixXBlockRowRef Ct, VectorXBlockRowRef delta)
{
  Eigen::HouseholderQR<Eigen::Ref<MatrixX>> QR(Cf);

  MatrixX Q = MatrixX::Zero(Cf.rows(), Cf.rows() - Cf.cols());
  Q.block(Cf.cols(), 0, Cf.rows() - Cf.cols(), Cf.rows() - Cf.cols()) =
      MatrixX::Identity(Cf.rows() - Cf.cols(), Cf.rows() - Cf.cols());
  Q.applyOnTheLeft(QR.householderQ());
  Q.transposeInPlace();

  Ct.block(0, 0, Q.rows(), Ct.cols()) = Q * Ct;
  delta.segment(0, Q.rows()) = Q * delta;
}

void UpdaterHelper::updateQRCompression(MatrixX& C, VectorX& delta)
{
  Eigen::HouseholderQR<MatrixX> QR(C.rows(), C.cols());
  QR.compute(C);

  MatrixX Q = MatrixX::Zero(C.rows(), C.cols());
  Q.block(0, 0, C.cols(), C.cols()) = MatrixX::Identity(C.cols(), C.cols());
  Q.applyOnTheLeft(QR.householderQ());

  C = QR.matrixQR().topRows(Q.cols()).triangularView<Eigen::Upper>();
  delta = Q.transpose() * delta;
}

bool UpdaterHelper::chi2Test(const fp& chi2, const size_t& dof, const std::map<uint, fp>& chi2_table)
{
  fp chi2_threshold;
  if (dof < 1000)
  {
    chi2_threshold = chi2_table.at(dof);
  }
  else
  {
    boost::math::chi_squared chi_squared_dist(dof);
    chi2_threshold = boost::math::quantile(chi_squared_dist, 0.95);
  }

  if (chi2 > chi2_threshold)
  {
    return false;
  }

  return true;
}

}  // namespace msceqf