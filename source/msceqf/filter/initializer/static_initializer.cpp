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
StaticInitializer::StaticInitializer(const InitializerOptions& opts) : opts_(opts) {}

void StaticInitializer::insertImu(const Imu& imu)
{
  if (imu_buffer_.empty() || imu.timestamp_ > imu_buffer_.back().timestamp_)
  {
    imu_buffer_.push_back(imu);
  }
  else
  {
    utils::Logger::warn("Received IMU measurement older then newest IMU measurement in buffer. Discarding measurement");
  }

  if (imu_buffer_.size() > 1 &&
      (imu_buffer_.back().timestamp_ - imu_buffer_.front().timestamp_) > opts_.imu_init_window_)
  {
    imu_buffer_.pop_front();
  }
}

bool StaticInitializer::detectMotion(/* camera meas */)
{
  return accelerationCheck() && disparityCheck(/* camera meas */);
}

bool StaticInitializer::accelerationCheck()
{
  if (opts_.acc_threshold_ > 0 &&
      (imu_buffer_.back().timestamp_ - imu_buffer_.front().timestamp_) < opts_.imu_init_window_)
  {
    utils::Logger::info("Not enough imu measurement for acceleration check in static initializer");
    return false;
  }

  Vector3 acc_mean = Vector3::Zero();
  for (const auto& imu : imu_buffer_)
  {
    acc_mean += imu.acc_;
  }

  std::vector<fp> acc_diff_norm_square;
  std::transform(imu_buffer_.begin(), imu_buffer_.end(), std::back_inserter(acc_diff_norm_square),
                 [&acc_mean](Imu& imu) { return (imu.acc_ - acc_mean).dot(imu.acc_ - acc_mean); });
  fp acc_std = std::reduce(acc_diff_norm_square.begin(), acc_diff_norm_square.end());

  if (acc_std < opts_.acc_threshold_)
  {
    utils::Logger::info("No accelerometer spike detected");
    return false;
  }

  return true;
}

bool StaticInitializer::disparityCheck(/* camera meas */) { return true; }

}  // namespace msceqf