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

#include "msceqf/filter/propagator/propagator.hpp"

#include <unsupported/Eigen/MatrixFunctions>
#include <utility>

#include "msceqf/symmetry/symmetry.hpp"
#include "utils/logger.hpp"

namespace msceqf
{
Propagator::Propagator(const PropagatorOptions& opts)
    : imu_buffer_()
    , Q_(Matrix12::Zero())
    , state_transition_order_(opts.state_transition_order_)
    , imu_buffer_max_size_(opts.imu_buffer_max_size_)
{
  // Assing continuous time process noice covariance
  Q_.block<3, 3>(0, 0) = std::pow(opts.angular_velocity_std_, 2) * Matrix3::Identity();
  Q_.block<3, 3>(3, 3) = std::pow(opts.acceleration_std_, 2) * Matrix3::Identity();
  Q_.block<3, 3>(6, 6) = std::pow(opts.angular_velocity_bias_std_, 2) * Matrix3::Identity();
  Q_.block<3, 3>(9, 9) = std::pow(opts.acceleration_bias_std_, 2) * Matrix3::Identity();
}

void Propagator::insertImu(MSCEqFState& X, const SystemState& xi0, const Imu& imu, fp& timestamp)
{
  if (imu_buffer_.empty() || imu.timestamp_ > imu_buffer_.back().timestamp_)
  {
    imu_buffer_.push_back(imu);
  }
  else
  {
    utils::Logger::warn("Received IMU measurement older then newest IMU measurement in buffer. Discarding measurement");
  }

  if (imu_buffer_.size() == imu_buffer_max_size_)
  {
    utils::Logger::warn("Maximum imu buffer size reached. Propagating and clearing the buffer");
    propagate(X, xi0, timestamp, imu_buffer_.back().timestamp_);
  }
}

Propagator::ImuBuffer Propagator::getImuReadings(const fp& t0, const fp& t1)
{
  // {
  //   std::stringstream ss;
  //   ss << imu_buffer_;
  //   utils::Logger::debug("IMU buffer before selection: " + ss.str());
  // }

  Propagator::ImuBuffer readings;

  auto first = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), t0);
  auto last = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), t1);

  if ((first == imu_buffer_.end() && last == imu_buffer_.end()) ||
      (first == imu_buffer_.begin() && last == imu_buffer_.begin()))
  {
    utils::Logger::err("No IMU readings in between " + std::to_string(t0) + " and " + std::to_string(t1));
    return readings;
  }

  // First IMU reading for integration checks
  // If the timestamp does not match t0, and if we have a measurement prior t0, then perform linear interpolation.
  if (first != imu_buffer_.begin() && first->timestamp_ > t0)
  {
    fp alpha = (t0 - (first - 1)->timestamp_) / (first->timestamp_ - (first - 1)->timestamp_);

    // utils::Logger::debug("First IMU reading interpolation between (" + std::to_string((first - 1)->timestamp_) + ", "
    // +
    //                      std::to_string(first->timestamp_) + "), at (" + std::to_string(t0) +
    //                      "), with alpha = " + std::to_string(alpha));

    readings.insert(readings.end(), lerp(*(first - 1), *first, alpha));
  }

  // If last is one past the end of the imu buffer then take all the previous readings for integration and keep the last
  // for future interpolation. If the last is not one past the end of imu buffer then take all the previous readings for
  // integration and if possible perform linear interpolation for future propagation
  if (last != imu_buffer_.end())
  {
    readings.insert(readings.end(), first, last);

    if (last != imu_buffer_.begin() && last->timestamp_ > t1)
    {
      fp alpha = (t1 - (last - 1)->timestamp_) / (last->timestamp_ - (last - 1)->timestamp_);

      // utils::Logger::debug("Last IMU reading interpolation between (" + std::to_string((last - 1)->timestamp_) + ", "
      // +
      //                      std::to_string(last->timestamp_) + "), at (" + std::to_string(t1) +
      //                      "), with alpha = " + std::to_string(alpha));

      msceqf::Imu new_first = lerp(*(last - 1), *last, alpha);

      // First erase to avoid invalidating the iterator
      imu_buffer_.erase(imu_buffer_.begin(), last);

      // Then push front the interpolated reading
      imu_buffer_.push_front(new_first);
    }
    else
    {
      imu_buffer_.erase(imu_buffer_.begin(), last);
    }
  }
  else
  {
    readings.insert(readings.end(), first, last);
    imu_buffer_.erase(imu_buffer_.begin(), last);
    imu_buffer_.push_back(readings.back());
  }

  // {
  //   std::stringstream ss;
  //   ss << imu_buffer_;
  //   utils::Logger::debug("IMU buffer after selection: " + ss.str());
  // }
  // {
  //   std::stringstream ss;
  //   ss << readings;
  //   utils::Logger::debug("IMU propagation readings: " + ss.str());
  // }

  return readings;
}

Imu Propagator::lerp(const Imu& pre, const Imu& post, const fp& alpha)
{
  assert(alpha > 0 && alpha < 1);

  Imu interp;

  interp.timestamp_ = (1 - alpha) * pre.timestamp_ + alpha * post.timestamp_;
  interp.ang_ = (1 - alpha) * pre.ang_ + alpha * post.ang_;
  interp.acc_ = (1 - alpha) * pre.acc_ + alpha * post.acc_;

  return interp;
}

bool Propagator::propagate(MSCEqFState& X, const SystemState& xi0, fp& timestamp, const fp& new_timestamp)
{
  assert(new_timestamp > timestamp);

  if (imu_buffer_.size() < 1)
  {
    utils::Logger::err("not enough IMU measurements in buffer to propagate with");
    return false;
  }

  Propagator::ImuBuffer propagation_buffer = getImuReadings(timestamp, new_timestamp);

  if (propagation_buffer.empty())
  {
    utils::Logger::err("Unable to propagate. Empty propagation buffer");
    return false;
  }

  for (auto it = propagation_buffer.begin(); it < propagation_buffer.end(); ++it)
  {
    bool is_last = it == (propagation_buffer.end() - 1);

    fp dt = is_last ? new_timestamp - it->timestamp_ : (it + 1)->timestamp_ - it->timestamp_;

    if (dt < eps_)
    {
      utils::Logger::err("ZERO dt (" + std::to_string(dt) + ") in propagation, skip propagation step");
      continue;
    }

    // utils::Logger::debug("Propagating with dt = " + std::to_string(dt) + "s");

    // Propagate covariance
    propagateCovariance(X, xi0, *it, dt);

    // Propagate mean
    propagateMean(X, xi0, *it, dt);
  }

  // Update timestamp
  timestamp = new_timestamp;

  return true;
}

void Propagator::propagateMean(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt)
{
  // Compute the Lift lambda
  SystemState::SystemStateAlgebraMap lambda = Symmetry::lift(Symmetry::phi(X, xi0), u);

  // Propagate mean
  X.state_.at(MSCEqFStateElementName::Dd)
      ->updateRight(
          dt * (Vector15() << lambda.at(SystemStateElementName::T), lambda.at(SystemStateElementName::b)).finished());

  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    X.state_.at(MSCEqFStateElementName::E)->updateRight(dt * lambda.at(SystemStateElementName::S));
  }
  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    X.state_.at(MSCEqFStateElementName::L)->updateRight(dt * lambda.at(SystemStateElementName::K));
  }
  // for (const auto& id : feat_ids)
  // {
  //   X.state_.at(id)->updateRight(dt * lambda.at(id));
  // }
}

void Propagator::propagateCovariance(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt)
{
  MatrixX Phi_core = coreStateTransitionMatrix(X, xi0, u, dt);

  // Core covariance propagation Phi * Sigma * Phi^T
  X.cov_.block(0, 0, Phi_core.rows(), Phi_core.cols()) =
      Phi_core * X.cov_.block(0, 0, Phi_core.rows(), Phi_core.cols()) * Phi_core.transpose();

  // Cross covariance propagation
  X.cov_.block(0, Phi_core.cols(), Phi_core.rows(), X.cov_.cols() - Phi_core.cols()) =
      Phi_core * X.cov_.block(0, Phi_core.cols(), Phi_core.rows(), X.cov_.cols() - Phi_core.cols());
  X.cov_.block(Phi_core.rows(), 0, X.cov_.rows() - Phi_core.rows(), Phi_core.cols()) =
      X.cov_.block(Phi_core.rows(), 0, X.cov_.rows() - Phi_core.rows(), Phi_core.cols()) * Phi_core.transpose();

  // Add discrete time processs noise covariance
  MatrixX M = inputMatrix(X, xi0, dt);
  X.cov_.block(0, 0, M.rows(), M.cols()) += M;
}

MatrixX Propagator::coreStateTransitionMatrix(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt)
{
  int size = 15;
  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    size += 6;
  }

  MatrixX A = MatrixX::Zero(size, size);

  // Precompute adjoint b0
  Matrix6 adb0 = SE3::adjoint(xi0.b());

  // Precompute Adjoint B and its inverse
  Matrix6 AdB = X.B().Adjoint();

  // Precompute theta
  Vector6 theta = AdB * u.w() + (Vector6() << Vector3::Zero(), (xi0.T().R().transpose() * xi0.ge3())).finished();

  // Precompute Psi matrix
  Matrix6 Psi = Matrix6::Zero();
  Psi.block(3, 0, 3, 3) = SO3::wedge(xi0.T().R().transpose() * xi0.ge3());

  // Compute A1
  A.block(0, 0, 6, 6) = Psi - adb0;
  A.block(6, 0, 3, 3) =
      SO3::wedge(xi0.T().R().transpose() * xi0.T().v()) - SO3::wedge(X.D().p()) * SO3::wedge(xi0.b().segment<3>(0));
  A.block(6, 3, 3, 3) = Matrix3::Identity();

  // Compute A2
  A.block(0, 9, 6, 6) = Matrix6::Identity();
  A.block(6, 9, 3, 3) = SO3::wedge(X.D().p());

  // Compute A3
  A.block(9, 0, 6, 6) = adb0 * Psi - SE3::adjoint(X.delta() + theta) * adb0;

  // Compute A4
  A.block(9, 9, 6, 6) = SE3::adjoint(X.delta() + theta);

  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    // Precompute Adjoint S0 inverse
    Matrix6 AdS0inv = xi0.S().invAdjoint();

    // Precompute the psi vector
    Vector3 psi1 = X.D().R() * u.ang_ + X.delta().segment<3>(0);
    Vector3 psi2 = psi1 - xi0.b().segment<3>(0);
    Vector3 psi3 = X.D().v() + SO3::wedge(X.D().p()) * psi1;
    Vector3 psi4 = X.D().v() + SO3::wedge(X.D().p()) * psi2 + xi0.T().R().transpose() * xi0.T().v();

    // Precompute rho
    Vector6 rho = (Vector6() << psi2, psi4).finished();

    // Precompute Xi matrix
    Eigen::Matrix<fp, 6, 9> Xi = Eigen::Matrix<fp, 6, 9>::Zero();
    Xi.block<3, 3>(0, 0) = -SO3::wedge(psi1);
    Xi.block<3, 3>(3, 0) = -SO3::wedge(psi3) - SO3::wedge(xi0.b().segment<3>(0)) * SO3::wedge(X.D().p());
    Xi.block<3, 3>(3, 3) = Matrix3::Identity();
    Xi.block<3, 3>(3, 6) = -SO3::wedge(psi2);

    // Precompute the Gamma matrix
    Matrix6 Gamma = Matrix6::Zero();
    Gamma.block<3, 3>(0, 0) = Matrix3::Identity();
    Gamma.block<3, 3>(3, 0) = SO3::wedge(X.D().p());

    // Compute A5
    A.block(15, 0, 6, 9) = AdS0inv * Xi;

    // Compute A6
    A.block(15, 9, 6, 6) = AdS0inv * Gamma;

    // Compute A7
    A.block(15, 15, 6, 6) = SE3::adjoint(AdS0inv * rho);
  }

  if (state_transition_order_ == 1)
  {
    return MatrixX::Identity(size, size) + A * dt;
  }
  if (state_transition_order_ == 2)
  {
    return MatrixX::Identity(size, size) + (A * dt) +
           coreSecondOrderPhi(A, dt, X.opts().enable_camera_extrinsics_calibration_);
  }
  else
  {
    // utils::Logger::debug("Computing state transition matrix via matrix exponential");
    return (A * dt).exp();
  }
}

MatrixX Propagator::coreSecondOrderPhi(const MatrixX& A, const fp& dt, const bool& enable_camera_extrinsics_calibration)
{
  MatrixX Phi_second = MatrixX::Zero(A.rows(), A.cols());

  // 11 block
  Phi_second.block(0, 0, 9, 9) =
      (A.block(0, 0, 9, 9).pow(2) + A.block(0, 9, 9, 6) * A.block(9, 0, 6, 9)) * std::pow(dt, 2) / 2;

  // 12 block
  Phi_second.block(0, 9, 9, 6) =
      ((A.block(0, 0, 9, 9) * A.block(0, 9, 9, 6)) + (A.block(0, 9, 9, 6) * A.block(9, 9, 6, 6))) * std::pow(dt, 2) / 2;

  // 21 block
  Phi_second.block(9, 0, 6, 9) =
      ((A.block(9, 0, 6, 9) * A.block(0, 0, 9, 9)) + (A.block(9, 9, 6, 6) * A.block(9, 0, 6, 9))) * std::pow(dt, 2) / 2;

  // 22 block
  Phi_second.block(9, 9, 6, 6) =
      (A.block(9, 9, 6, 6).pow(2) + A.block(9, 0, 6, 9) * A.block(0, 9, 9, 6)) * std::pow(dt, 2) / 2;

  if (enable_camera_extrinsics_calibration)
  {
    // 31 block
    Phi_second.block(15, 0, 6, 9) =
        ((A.block(15, 0, 6, 9) * A.block(0, 0, 9, 9)) + (A.block(15, 9, 6, 6) * A.block(9, 0, 6, 9)) +
         (A.block(15, 15, 6, 6) * A.block(15, 0, 6, 9))) *
        std::pow(dt, 2) / 2;

    // 31 block
    Phi_second.block(15, 9, 6, 6) =
        ((A.block(15, 0, 6, 9) * A.block(0, 9, 9, 6)) + (A.block(15, 9, 6, 6) * A.block(9, 9, 6, 6)) +
         (A.block(15, 15, 6, 6) * A.block(15, 9, 6, 6))) *
        std::pow(dt, 2) / 2;

    // 33 block
    Phi_second.block(15, 15, 6, 6) = A.block(15, 15, 6, 6).pow(2) * std::pow(dt, 2) / 2;
  }

  return Phi_second;
}

MatrixX Propagator::inputMatrix(MSCEqFState& X, const SystemState& xi0, const fp& dt)
{
  int size = 15;
  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    size += 6;
  }

  MatrixX B = MatrixX::Zero(size, 12);

  Eigen::Matrix<fp, 9, 6> B1 = Eigen::Matrix<fp, 9, 6>::Zero();
  B1.block<6, 6>(0, 0) = Matrix6::Identity();

  Matrix9 AdD = X.D().Adjoint();

  B.block(0, 0, 9, 6) = AdD * B1;
  B.block(9, 0, 6, 6) = SE3::adjoint(xi0.b()) * AdD.block<6, 6>(0, 0);
  B.block(9, 6, 6, 6) = -AdD.block<6, 6>(0, 0);

  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    Matrix6 B2 = Matrix6::Zero();
    B2.block<3, 3>(0, 0) = Matrix3::Identity();

    B.block(15, 0, 6, 6) = (xi0.S().inv() * X.C()).Adjoint() * B2;
  }

  return (B * Q_ * B.transpose()) * dt;
}

}  // namespace msceqf
