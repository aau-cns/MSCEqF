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

#include "msceqf/msceqf.hpp"

#include <future>

namespace msceqf
{

MSCEqF::MSCEqF(const std::string& params_filepath)
    : parser_(params_filepath)
    , opts_(parser_.parseOptions())
    , X_(opts_.state_options_)
    , xi0_(opts_.state_options_)
    , track_manager_(opts_.track_manager_options_, opts_.state_options_.initial_camera_intrinsics_.k())
    , initializer_(opts_.init_options_)
    , propagator_(opts_.propagator_options_)
{
}

void MSCEqF::processImuMeasurement(const Imu& imu)
{
  assert(imu.timestamp_ >= 0);

  if (!is_filter_initialized_)
  {
    utils::Logger::info("Collecting IMU measurements for static initialization.");
    initializer_.insertImu(imu);
    return;
  }

  if (imu.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received IMU measurement older than actual state estimate. Discarding measurement.");
  }
  else
  {
    propagator_.insertImu(X_, xi0_, imu, timestamp_);
  }
}

void MSCEqF::processCameraMeasurement(Camera& cam)
{
  assert(cam.timestamp_ >= 0);
  assert(cam.image_.size() == cam.mask_.size());

  if (!is_filter_initialized_)
  {
    track_manager_.processCamera(cam);
    is_filter_initialized_ = initializer_.detectMotion(track_manager_.tracks());
  }

  if (cam.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Camera measurement older than actual state estimate. Discarding measurement.");
    return;
  }

  // Parallelize propagation and image processing, synchronization is not needed since there are no data races
  auto future_propagation = std::async([&]() { return propagator_.propagate(X_, xi0_, timestamp_, cam.timestamp_); });
  auto future_image_processing = std::async([&]() { track_manager_.processCamera(cam); });

  if (!future_propagation.get())
  {
    utils::Logger::err("Propagation failure.");
    return;
  }

  // Parallelize stochastic cloning and image processing, synchronization is not needed since there are no data races
  auto future_cloning = std::async([&]() { X_.stochasticCloning(cam.timestamp_); });

  future_image_processing.wait();
  future_cloning.wait();

  track_manager_.lostTracksIds(cam.timestamp_, ids_to_update_);

  bool marginalize = false;
  fp marginalize_timestamp = -1;
  if (X_.clonesSize() == opts_.state_options_.num_clones_)
  {
    marginalize_timestamp = X_.cloneTimestampToMarginalize();
    track_manager_.activeTracksIds(marginalize_timestamp, ids_to_update_);
    marginalize = true;
  }

  // Update
  // updater_.update(X, xi0, ids_to_update_, track_manager_.tracks())

  if (marginalize)
  {
    X_.marginalizeCloneAt(marginalize_timestamp);
  }

  // Remove tracks used for update and clear ids_to_update_
  track_manager_.removeTracksId(ids_to_update_);
  ids_to_update_.clear();
}

const MSCEqFOptions& MSCEqF::options() const { return opts_; }

const StateOptions& MSCEqF::stateOptions() const { return opts_.state_options_; }

const MatrixX& MSCEqF::Covariance() const { return X_.Cov(); }

const SystemState MSCEqF::stateEstimate() const { return Symmetry::phi(X_, xi0_); }

}  // namespace msceqf