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

void ProjectionHelperS2::innovationBlock([[maybe_unused]] const MSCEqFState& X,
                                         [[maybe_unused]] const SystemState& xi0,
                                         [[maybe_unused]] const FeatHelper& feat,
                                         [[maybe_unused]] MatrixXBlockRowRef C_block_row,
                                         [[maybe_unused]] VectorXBlockRowRef delta_block_row,
                                         [[maybe_unused]] MatrixXBlockRowRef Cf_block_row)
{
  throw std::runtime_error("Update with S2 projection not implemented yet");
}

void ProjectionHelperZ1::innovationBlock(const MSCEqFState& X,
                                         const SystemState& xi0,
                                         const FeatHelper& feat,
                                         MatrixXBlockRowRef C_block_row,
                                         VectorXBlockRowRef delta_block_row,
                                         MatrixXBlockRowRef Cf_block_row)
{
  // Here feat.A_f_ is the feature in the global frame (groundturth)
  Vector3 G0_f = (xi0.P() * xi0.S()).inv() * feat.A_f_;

  // Vector3 G0_f = feat.anchor_->E_ * feat.A_f_;

  // feature in origin frmae and camera frame
  Vector3 C_f = feat.clone_->E_.inv() * G0_f;

  // precompute D = K0 * L * dpi(C_f) if intrinsics are calibrated, D = dpi(C_f) otherwise
  Eigen::Matrix<fp, 2, 3> D = X.opts_.enable_camera_intrinsics_calibration_ ?
                                  (xi0.K() * X.L()).asMatrix().block<2, 2>(0, 0) * dpi(C_f) :
                                  dpi(C_f);

  // Precompute P = L * pi(C_f) if intrinsics are calibrated, P = pi(C_f) otherwise
  Vector3 P = X.opts_.enable_camera_intrinsics_calibration_ ? X.L().asMatrix() * pi(C_f) : pi(C_f);

  if (X.opts_.enable_camera_extrinsics_calibration_)
  {
    Eigen::Matrix<fp, 3, 6> A = Eigen::Matrix<fp, 3, 6>::Zero();
    A.block<3, 3>(0, 0) = SO3::wedge(G0_f);
    A.block<3, 3>(0, 3) = -Matrix3::Identity();

    C_block_row.block(0, feat.clone_->getIndex(), block_rows_, 6).noalias() = D * feat.clone_->E_.R().transpose() * A;

    // if (feat.clone_->getIndex() != feat.anchor_->getIndex())
    // {
    //   C_block_row.block(0, feat.anchor_->getIndex(), block_rows_, 6).noalias() = -D * feat.clone_->E_.inv().R() * A;
    // }
  }
  else
  {
    // [TODO] Ct if extrinsics are not calibrated
  }

  if (X.opts_.enable_camera_intrinsics_calibration_)
  {
    C_block_row.block(0, X.stateElementIndex(MSCEqFStateElementName::L), block_rows_, 4).noalias() =
        xi0.K().asMatrix().block<2, 2>(0, 0) * UpdaterHelper::Xi(P);
  }

  switch (feature_representation_)
  {
    case FeatureRepresentation::EUCLIDEAN:
      Cf_block_row = D * (xi0.P().R() * xi0.S().R() * feat.clone_->E_.R()).transpose();
      break;
    case FeatureRepresentation::ANCHORED_INVERSE_DEPTH:
      // Cf_block_row = D * (xi0.P().R() * xi0.S().R() * feat.clone_->E_.R()).transpose() *
      //                UpdaterHelper::inverseDepthJacobian(feat.A_f_);
      throw std::runtime_error("Anchored inverse depth representation update not implemented");
      break;
    case FeatureRepresentation::ANCHORED_POLAR:
      // Cf_block_row = Eigen::Matrix<fp, 2, 4>::Zero();
      throw std::runtime_error("Anchored polar representation update not implemented");
      break;
    default:
      break;
  }

  // Residual
  if (X.opts_.enable_camera_intrinsics_calibration_)
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

void UpdaterHelper::updateQRCompression(MatrixX& C, VectorX& delta, MatrixX& R)
{
  Eigen::HouseholderQR<MatrixX> QR(C.rows(), C.cols());
  QR.compute(C);

  MatrixX Q = MatrixX::Zero(C.rows(), C.cols());
  Q.block(0, 0, C.cols(), C.cols()) = MatrixX::Identity(C.cols(), C.cols());
  Q.applyOnTheLeft(QR.householderQ());

  C = QR.matrixQR().topRows(Q.cols()).triangularView<Eigen::Upper>();
  delta = Q.transpose() * delta;
  R = Q.transpose() * R * Q;
}

bool UpdaterHelper::chi2Test(const MSCEqFState& X,
                             const MatrixXBlockRowRef Ct_block,
                             const VectorXBlockRowRef delta_block,
                             const fp& pixel_std,
                             const std::map<uint, fp>& chi2_table)
{
  const auto& dof = delta_block.rows();

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

  // MatrixX S = Ct_block * X.subCov(keys) * Ct_block.transpose();
  MatrixX S = Ct_block * X.cov() * Ct_block.transpose();
  S.diagonal() += Eigen::VectorXd::Ones(S.rows()) * pixel_std * pixel_std;
  fp chi2 = delta_block.dot(S.llt().solve(delta_block));

  // Compute chi2 threshold
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