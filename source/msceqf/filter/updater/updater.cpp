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
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "msceqf/filter/updater/updater.hpp"
#include "msceqf/symmetry/symmetry.hpp"
#include "utils/logger.hpp"

namespace msceqf
{

Updater::Updater(const UpdaterOptions& opts, const SystemState& xi0)
    : opts_(opts), xi0_(xi0), ph_(nullptr), chi2_table_(), update_ids_(), total_size_(0)
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

void Updater::mscUpdate(MSCEqFState& X, const Tracks& tracks, std::unordered_set<uint>& ids)
{
  if (ids.empty())
  {
    utils::Logger::warn("No tracks to update with");
    return;
  }

  // Precompute the total number of rows of the C matrix and the residual delta with a "safe" margin
  size_t rows = 0;
  size_t long_tracks = 0;
  for (const auto& id : ids)
  {
    if (tracks.at(id).size() < opts_.min_track_lenght_)
    {
      continue;
    }
    rows += tracks.at(id).size();
    ++long_tracks;
  }
  rows = rows * ph_->block_rows() - ph_->dim_loss() * (long_tracks - 1);

  // Reset the map of columns of the C matrix for each variable involved in the update
  cols_map_.clear();

  // Fill the map of the indices of the columns of the C matrix for the variable involved in it, with insertion order.
  // This is needed for block operations. On doing so, we precompute the total number of columns of the C matrix.
  size_t cols = 0;
  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    cols_map_.insert(MSCEqFStateElementName::L, cols);
    cols += X.dof(MSCEqFStateElementName::L);
  }
  for (const auto& [timestamp, clone] : X.clones_)
  {
    cols_map_.insert(timestamp, cols);
    cols += clone->getDof();
  }

  // Preallocate C matrix and residual delta
  MatrixX C = MatrixX::Zero(rows, cols);
  VectorX delta = VectorX::Zero(rows);

  // Reset vector of ids that will be actually used in the update, and the effective size of C and residual delta
  update_ids_.clear();
  total_size_ = 0;

  // For each track triangulate the feature, compute C and delta blocks, and performe chi2 rejection test
  for (const auto& id : ids)
  {
    const auto& track = tracks.at(id);

    if (track.size() < opts_.min_track_lenght_)
    {
      utils::Logger::debug("Track with id: " + std::to_string(id) + " do not contain enough views for triangulation");
      continue;
    }
    const auto& track_size = track.size();

    // Triangulate feature in anchor frame (frame of first observation of the feature)
    const auto& A_E = X.clone(track.timestamps_.front());
    Vector3 A_f = Vector3::Zero();

    if (!linearTriangulation(X, track, A_E, A_f))
    {
      utils::Logger::debug("Linear triangulation failed for track id: " + std::to_string(id));
      continue;
    }

    if (opts_.refine_traingulation_)
    {
      utils::Logger::debug(
          "Linear triangulation succeeded. Nonlinear triangulation for track id: " + std::to_string(id) + "...");
      nonlinearTriangulation(X, track, A_E, A_f);
    }

    // For each feature measurement in track compute the Jacobian and residual block
    // (C matrix block, Cf matrix block and delta block)
    MatrixX Cf = MatrixX::Zero(ph_->block_rows() * track_size, ph_->dim_loss());
    for (size_t i = 0; i < track_size; ++i)
    {
      Vector2 uv(track.uvs_[i].x, track.uvs_[i].y);
      Vector2 uvn(track.normalized_uvs_[i].x, track.normalized_uvs_[i].y);
      FeatHelper feat(A_f, uv, uvn, track.timestamps_.front(), track.timestamps_[i]);

      const auto& row_idx = total_size_ + (ph_->block_rows() * i);
      C.middleRows(row_idx, ph_->block_rows()).setZero();
      delta.middleRows(row_idx, ph_->block_rows()).setZero();

      ph_->residualJacobianBlock(X, xi0_, feat, C.middleRows(row_idx, ph_->block_rows()),
                                 delta.middleRows(row_idx, ph_->block_rows()),
                                 Cf.middleRows(ph_->block_rows() * i, ph_->block_rows()), cols_map_);
    }

    UpdaterHelper::nullspaceProjection(Cf, C.middleRows(total_size_, ph_->block_rows() * track_size),
                                       delta.middleRows(total_size_, ph_->block_rows() * track_size));

    const auto& C_block = C.middleRows(total_size_, (ph_->block_rows() * track_size) - ph_->dim_loss());
    const auto& delta_block = delta.middleRows(total_size_, (ph_->block_rows() * track_size) - ph_->dim_loss());

    MatrixX S = C_block * X.subCov(cols_map_.keys()) * C_block.transpose();
    S.diagonal() += Eigen::VectorXd::Ones(S.rows()) * opts_.pixel_std_ * opts_.pixel_std_;
    fp chi2 = delta_block.dot(S.llt().solve(delta_block));

    if (!UpdaterHelper::chi2Test(chi2, (ph_->block_rows() * track_size) - ph_->dim_loss(), chi2_table_))
    {
      utils::Logger::debug("Chi2 test failed for track id: " + std::to_string(id));
      continue;
    }

    // Update total size of C matrix and residual delta
    total_size_ += (ph_->block_rows() * track_size) - ph_->dim_loss();

    // Add id to vector of ids that will be used in the update
    update_ids_.emplace_back(id);
  }

  if (update_ids_.empty())
  {
    ids.clear();
    utils::Logger::warn("No valid features to update with. Skipping update step");
    return;
  }
  else
  {
    // Keep only ids that will be used in the update
    for (auto it = ids.begin(); it != ids.end();)
    {
      if (find(update_ids_.begin(), update_ids_.end(), *it) == update_ids_.end())
      {
        it = ids.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  // Resize residual delta and C matrix based on total_size_
  delta.conservativeResize(total_size_);
  C.conservativeResize(total_size_, C.cols());

  // Define measurement noise covariance
  MatrixX R = MatrixX::Identity(C.rows(), C.rows()) * opts_.pixel_std_ * opts_.pixel_std_;

  // Update compression
  if (C.rows() > C.cols())
  {
    UpdaterHelper::updateQRCompression(C, delta, R);
  }

  // MSCEqF Update
  UpdateMSCEqF(X, C, delta, R);
}

bool Updater::linearTriangulation(const MSCEqFState& X, const Track& track, const SE3& A_E, Vector3& A_f) const
{
  Matrix3 A = Matrix3::Zero();
  Vector3 b = Vector3::Zero();

  Matrix3 Ai = Matrix3::Zero();
  Vector3 A_bf = Vector3::Zero();

  std::vector<Vector3> bearings;

  for (size_t i = 0; i < track.size(); ++i)
  {
    SE3 E = A_E.inv() * X.clone(track.timestamps_[i]);

    A_bf(0) = track.normalized_uvs_[i].x;
    A_bf(1) = track.normalized_uvs_[i].y;
    A_bf(2) = 1.0;

    A_bf = E.R() * A_bf;
    Ai = -SO3::wedge(A_bf) * SO3::wedge(A_bf);

    A += Ai;
    b += Ai * E.x();

    bearings.push_back(A_bf.normalized());
  }

  A_f = A.colPivHouseholderQr().solve(b);

  fp min_cos = 1;
  for (size_t i = 0; i < bearings.size(); ++i)
  {
    for (size_t j = i + 1; j < bearings.size(); ++j)
    {
      fp cos = bearings[i].dot(bearings[j]);
      if (cos < min_cos)
      {
        min_cos = cos;
      }
    }
  }
  fp max_angle = std::acos(min_cos) * 180 / M_PI;

  // // Comparison with OpenCV triangulation
  // cv::Mat uvn_first_cv = (cv::Mat_<fp>(2, 1) << track.normalized_uvs_.front().x, track.normalized_uvs_.front().y);
  // cv::Mat uvn_last_cv = (cv::Mat_<fp>(2, 1) << track.normalized_uvs_.back().x, track.normalized_uvs_.back().y);

  // Eigen::Matrix<fp, 3, 4> P_first_eigen = Eigen::Matrix<fp, 3, 4>::Zero();
  // P_first_eigen.block(0, 0, 3, 3) = Matrix3::Identity();

  // Eigen::Matrix<fp, 3, 4> P_last_eigen = (X.clone(track.timestamps_.back()).inv() * A_E).asMatrix().block(0, 0, 3,
  // 4);

  // cv::Mat P_first_cv, P_last_cv;
  // cv::eigen2cv(P_first_eigen, P_first_cv);
  // cv::eigen2cv(P_last_eigen, P_last_cv);

  // cv::Mat A_f_cv;
  // cv::triangulatePoints(P_first_cv, P_last_cv, uvn_first_cv, uvn_last_cv, A_f_cv);
  // Vector3 A_f_eigen = Vector3(A_f_cv.at<fp>(0, 0) / A_f_cv.at<fp>(3, 0), A_f_cv.at<fp>(1, 0) / A_f_cv.at<fp>(3, 0),
  //                             A_f_cv.at<fp>(2, 0) / A_f_cv.at<fp>(3, 0));

  // utils::Logger::debug("A_f with custom LS: " +
  //                      static_cast<std::ostringstream&>(std::ostringstream() << A_f.transpose()).str());
  // utils::Logger::debug("A_f with OpenCV: " +
  //                      static_cast<std::ostringstream&>(std::ostringstream() << A_f_eigen.transpose()).str());

  if (A_f(2) < opts_.min_depth_ || A_f(2) > opts_.max_depth_ || std::isnan(A_f.norm()) || max_angle < opts_.min_angle_)
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
    utils::Logger::debug("Feature refinement not converged, reached max iterations");
    if (actual_res_norm > initial_residual_norm)
    {
      A_f = A_f_init;
      return;
    }
  }

  // Return given initial value if invalid
  if (A_f(2) < opts_.min_depth_ || A_f(2) > opts_.max_depth_ || std::isnan(A_f.norm()))
  {
    utils::Logger::debug("Feature refinement converged to invalid value");
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

  Vector3 Gf0 = A_E * A_f;

  for (size_t i = 0; i < track.size(); ++i)
  {
    const auto& clone_E = X.clone(track.timestamps_[i]);

    Vector3 Ci_f = clone_E.inv() * Gf0;
    Vector3 Ci_f_invdepth(Ci_f(0) / Ci_f(2), Ci_f(1) / Ci_f(2), 1 / Ci_f(2));

    Vector2 uvn(track.normalized_uvs_[i].x, track.normalized_uvs_[i].y);
    res.block(2 * i, 0, 2, 1) = uvn - Ci_f_invdepth.block(0, 0, 2, 1);

    J.block(2 * i, 0, 2, 2) = Matrix2::Identity();
    J.block(2 * i, 2, 2, 1) = -Ci_f_invdepth.block<2, 1>(0, 0);
    J.block(2 * i, 0, 2, 3) = Ci_f_invdepth(2) * J.block(2 * i, 0, 2, 3) * clone_E.R().transpose() * A_E.R() * J_rep;
  }
}

void Updater::UpdateMSCEqF(MSCEqFState& X, const MatrixX& C, const VectorX& delta, const MatrixX& R) const
{
  // Compute Kalman gain and innovation
  MatrixX G = X.subCovCols(cols_map_.keys()) * C.transpose();
  MatrixX S(R.rows(), R.cols());
  S.triangularView<Eigen::Upper>() = C * X.subCov(cols_map_.keys()) * C.transpose();
  S.triangularView<Eigen::Upper>() += R;
  MatrixX invS = MatrixX::Identity(R.rows(), R.cols());
  S.selfadjointView<Eigen::Upper>().ldlt().solveInPlace(invS);
  MatrixX K = G * invS.selfadjointView<Eigen::Upper>();
  VectorX inn = K * delta;

  assert((inn.segment(15, 6) - inn.segment(inn.rows() - 6, 6)).norm() < 1e-12);

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
  }
}

}  // namespace msceqf