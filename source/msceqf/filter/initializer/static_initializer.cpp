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

#include <algorithm>
#include <numeric>

#include "msceqf/filter/initializer/static_initializer.hpp"
#include "utils/logger.hpp"

namespace msceqf
{
StaticInitializer::StaticInitializer(const InitializerOptions& opts, const Checker& checker)
    : opts_(opts), checker_(checker), imu_buffer_(), T0_(), b0_(Vector6::Zero())
{
}

void StaticInitializer::insertImu(const Imu& imu)
{
  if (imu_buffer_.empty())
  {
    utils::Logger::info("Collecting IMU measurements for static initialization");
    imu_buffer_.push_back(imu);
    return;
  }

  if (imu.timestamp_ > imu_buffer_.back().timestamp_)
  {
    imu_buffer_.push_back(imu);
  }
  else
  {
    utils::Logger::warn("Received IMU measurement older then newest IMU measurement in buffer. Discarding measurement");
  }

  if (imu_buffer_.size() > 2 &&
      (imu_buffer_.back().timestamp_ - imu_buffer_.front().timestamp_) > (1.0 + opts_.imu_init_window_))
  {
    imu_buffer_.pop_front();
  }
}

bool StaticInitializer::detectMotion(const Tracks& tracks)
{
  return detectAccelerationSpike() && checker_.disparityCheck(tracks);
}

bool StaticInitializer::initializeOrigin()
{
  Vector3 acc_mean = Vector3::Zero();
  Vector3 ang_mean = Vector3::Zero();
  fp acc_std = 0.0;

  if (!imuMeanStd(acc_mean, ang_mean, acc_std))
  {
    utils::Logger::info("Not enough imu measurement for acceleration check in static initializer");
    return false;
  }

  computeOrigin(acc_mean, ang_mean);

  return true;
}

bool StaticInitializer::detectAccelerationSpike()
{
  if (opts_.acc_threshold_ <= 0 && !imu_buffer_.empty())
  {
    utils::Logger::info("Acceleration check in static initializer disabled");
    return true;
  }

  Vector3 acc_mean = Vector3::Zero();
  Vector3 ang_mean = Vector3::Zero();
  fp acc_std = 0.0;

  if (!imuMeanStd(acc_mean, ang_mean, acc_std))
  {
    utils::Logger::info("Not enough imu measurement for acceleration check in static initializer");
    return false;
  }

  if (acc_std < opts_.acc_threshold_)
  {
    utils::Logger::info("No accelerometer spike detected");
    return false;
  }

  computeOrigin(acc_mean, ang_mean);

  return true;
}

bool StaticInitializer::imuMeanStd(Vector3& acc_mean, Vector3& ang_mean, fp& acc_std) const
{
  if (imu_buffer_.size() < 2 ||
      (imu_buffer_.back().timestamp_ - imu_buffer_.front().timestamp_) < opts_.imu_init_window_)
  {
    return false;
  }

  for (const auto& imu : imu_buffer_)
  {
    acc_mean += imu.acc_;
    ang_mean += imu.ang_;
  }
  acc_mean /= imu_buffer_.size();
  ang_mean /= imu_buffer_.size();

  std::vector<fp> acc_diff_norm_square;
  acc_diff_norm_square.reserve(imu_buffer_.size());
  std::transform(imu_buffer_.begin(), imu_buffer_.end(), std::back_inserter(acc_diff_norm_square),
                 [&acc_mean](const Imu& imu) { return (imu.acc_ - acc_mean).dot(imu.acc_ - acc_mean); });
  acc_std = std::reduce(acc_diff_norm_square.begin(), acc_diff_norm_square.end());
  acc_std = std::sqrt(acc_std / (acc_diff_norm_square.size() - 1));

  return true;
}

void StaticInitializer::computeOrigin(Vector3& acc_mean, Vector3& ang_mean)
{
  Vector3 z = acc_mean / acc_mean.norm();

  Vector3 x = Vector3(1, 0, 0) - z * z.dot(Vector3(1, 0, 0));
  x = x / x.norm();

  Vector3 y = z.cross(x);
  y = y / y.norm();

  Matrix3 R0;
  R0.block<1, 3>(0, 0) = x.transpose();
  R0.block<1, 3>(1, 0) = y.transpose();
  R0.block<1, 3>(2, 0) = z.transpose();

  T0_ = SE23(R0, {Vector3::Zero(), Vector3::Zero()});

  if (opts_.identity_b0_)
  {
    utils::Logger::info("Bias origin set to identity");
    return;
  }

  b0_.segment<3>(0) = ang_mean;
  b0_.segment<3>(3) = acc_mean - R0.transpose() * (opts_.gravity_ * Vector3(0, 0, 1));
}

const SE23& StaticInitializer::T0() const { return T0_; }

const Vector6& StaticInitializer::b0() const { return b0_; }

}  // namespace msceqf
