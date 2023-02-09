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
                                         [[maybe_unused]] VectorXBlockRowRef res_block_row,
                                         [[maybe_unused]] MatrixXBlockRowRef Cf_block_row)
{
  throw std::runtime_error("Update with S2 projection not implemented yet");
}

void ProjectionHelperZ1::innovationBlock(const MSCEqFState& X,
                                         const SystemState& xi0,
                                         const FeatHelper& feat,
                                         MatrixXBlockRowRef C_block_row,
                                         VectorXBlockRowRef res_block_row,
                                         MatrixXBlockRowRef Cf_block_row)
{
  // feature in origin frmae and camera frame
  Vector3 G0_f = feat.A_E_ * feat.A_f_;
  Vector3 C_f = feat.clone_->E_.inv() * G0_f;

  // precompute some terms (K0 * L * dpi(C_f) and L * pi(C_f))
  Eigen::Matrix<fp, 2, 3> K0Ldpi = (xi0.K() * X.L()).asMatrix().block<2, 2>(0, 0) * dpi(C_f);
  Vector3 Lpi_homogenous = X.L().asMatrix() * pi(C_f);

  if (X.opts_.enable_camera_extrinsics_calibration_)
  {
    Eigen::Matrix<fp, 3, 6> A = Eigen::Matrix<fp, 3, 6>::Zero();
    A.block<3, 3>(0, 0) = SO3::wedge(G0_f);
    A.block<3, 3>(0, 3) = -Matrix3::Identity();

    const uint& col_idx = X.stateElementIndex(MSCEqFStateElementName::E);
    C_block_row.block(0, col_idx, block_rows_, 6).noalias() = K0Ldpi * X.E().inv().R() * A;
  }
  else
  {
    // [TODO] Ct if extrinsics are not calibrated
  }

  if (X.opts_.enable_camera_intrinsics_calibration_)
  {
    const uint& col_idx = X.stateElementIndex(MSCEqFStateElementName::L);
    C_block_row.block(0, col_idx, block_rows_, 4).noalias() =
        xi0.K().asMatrix().block<2, 2>(0, 0) * UpdaterHelper::Xi(Lpi_homogenous);
  }

  switch (feature_representation_)
  {
    case FeatureRepresentation::EUCLIDEAN:
      Cf_block_row = K0Ldpi * X.E().inv().R();
      break;
    case FeatureRepresentation::ANCHORED_INVERSE_DEPTH:
      Cf_block_row = K0Ldpi * (X.E().inv() * feat.A_E_).R() * UpdaterHelper::inverseDepthJacobian(feat.A_f_);
      break;
    case FeatureRepresentation::ANCHORED_POLAR:
      Cf_block_row = Eigen::Matrix<fp, 2, 4>::Zero();
      throw std::runtime_error("Anchored polar representation update not implemented");
      break;
    default:
      break;
  }

  // Residual
  res_block_row = feat.uvn_ - (xi0.K().asMatrix() * Lpi_homogenous).segment<2>(0);
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

void UpdaterHelper::nullspaceProjection(Eigen::Ref<MatrixX> Cf, MatrixXBlockRowRef Ct, VectorXBlockRowRef res)
{
  Eigen::HouseholderQR<Eigen::Ref<MatrixX>> QR(Cf);
  QR.compute(Cf);

  MatrixX Q = MatrixX::Identity(Cf.rows(), Cf.rows());
  Q.applyOnTheLeft(QR.householderQ());

  MatrixX A = Q.block(0, 0, Cf.rows(), Cf.cols()).transpose();

  Ct = A * Ct;
  res = A * res;
}

bool UpdaterHelper::chi2Test(const MSCEqFState& X,
                             const MatrixXBlockRowRef Ct_block,
                             const VectorXBlockRowRef res_block,
                             const fp& pixel_std,
                             const std::map<uint, fp>& chi2_table)
{
  const auto& dof = res_block.rows();

  // get covariance of variables involved in update
  std::vector<MSCEqFState::MSCEqFStateKey> keys;
  if (X.opts_.enable_camera_extrinsics_calibration_)
  {
    keys.push_back(MSCEqFStateElementName::E);
  }
  else
  {
    // [TODO]
  }
  if (X.opts_.enable_camera_intrinsics_calibration_)
  {
    keys.push_back(MSCEqFStateElementName::L);
  }

  MatrixX S = Ct_block * X.subCov(keys) * Ct_block.transpose();
  S.diagonal() += (pixel_std * pixel_std) * Eigen::VectorXd::Ones(S.rows());
  fp chi2 = res_block.dot(S.llt().solve(res_block));

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