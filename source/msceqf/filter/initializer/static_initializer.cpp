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

bool StaticInitializer::detectMotion(const Tracks& tracks) const
{
  return accelerationCheck() && disparityCheck(tracks);
}

bool StaticInitializer::accelerationCheck() const
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
  acc_diff_norm_square.reserve(imu_buffer_.size());
  std::transform(imu_buffer_.begin(), imu_buffer_.end(), std::back_inserter(acc_diff_norm_square),
                 [&acc_mean](const Imu& imu) { return (imu.acc_ - acc_mean).dot(imu.acc_ - acc_mean); });
  fp acc_std = std::reduce(acc_diff_norm_square.begin(), acc_diff_norm_square.end());

  if (acc_std < opts_.acc_threshold_)
  {
    utils::Logger::info("No accelerometer spike detected");
    return false;
  }

  return true;
}

bool StaticInitializer::disparityCheck(const Tracks& tracks) const
{
  const auto& longest_track = std::max_element(tracks.begin(), tracks.end(),
                                               [](const auto& pre, const auto& post) {
                                                 return pre.second.timestamps_.size() < post.second.timestamps_.size();
                                               })
                                  ->second;

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

}  // namespace msceqf