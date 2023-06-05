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

#include "msceqf/filter/initializer/static_initializer.hpp"

#include <algorithm>
#include <numeric>

#include "utils/logger.hpp"

namespace msceqf
{
StaticInitializer::StaticInitializer(const InitializerOptions& opts)
    : opts_(opts), imu_buffer_(), T0_(), b0_(Vector6::Zero())
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

bool StaticInitializer::detectMotion(const Tracks& tracks) { return accelerationCheck() && disparityCheck(tracks); }

bool StaticInitializer::accelerationCheck()
{
  if (opts_.acc_threshold_ <= 0 && !imu_buffer_.empty())
  {
    utils::Logger::info("Acceleration check in static initializer disabled");
    return true;
  }

  if (imu_buffer_.size() < 2 ||
      (imu_buffer_.back().timestamp_ - imu_buffer_.front().timestamp_) < opts_.imu_init_window_)
  {
    utils::Logger::info("Not enough imu measurement for acceleration check in static initializer");
    return false;
  }

  Vector3 acc_mean = Vector3::Zero();
  Vector3 ang_mean = Vector3::Zero();
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
  fp acc_std = std::reduce(acc_diff_norm_square.begin(), acc_diff_norm_square.end());
  acc_std = std::sqrt(acc_std / (acc_diff_norm_square.size() - 1));

  if (acc_std < opts_.acc_threshold_)
  {
    utils::Logger::info("No accelerometer spike detected");
    return false;
  }

  if (opts_.identity_xi0_)
  {
    utils::Logger::info("Origin set to identity");
    return true;
  }

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
  b0_.segment<3>(0) = ang_mean;
  b0_.segment<3>(3) = acc_mean - R0.transpose() * (opts_.gravity_ * Vector3(0, 0, 1));

  return true;
}

bool StaticInitializer::disparityCheck(const Tracks& tracks) const
{
  const auto& longest_track = std::max_element(tracks.begin(), tracks.end(), [](const auto& pre, const auto& post) {
                                return pre.second.timestamps_.size() < post.second.timestamps_.size();
                              })->second;

  const fp& longest_track_time = longest_track.timestamps_.back() - longest_track.timestamps_.front();

  if (longest_track_time < opts_.disparity_window_)
  {
    utils::Logger::info("feature tracks not long enough for disparity check in static initializer");
    return false;
  }

  fp average_disparity = 0;
  int track_cnt = 0;

  for (const auto& [id, track] : tracks)
  {
    if (track.size() == longest_track.size())
    {
      average_disparity += cv::norm(track.uvs_.back() - track.uvs_.front());
      ++track_cnt;
    }
  }
  average_disparity /= track_cnt;

  return average_disparity > opts_.disparity_threshold_ ? true : false;
}

const SE23& StaticInitializer::T0() const { return T0_; }

const Vector6& StaticInitializer::b0() const { return b0_; }

}  // namespace msceqf
