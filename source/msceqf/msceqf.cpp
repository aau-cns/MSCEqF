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
    , updater_(opts_.updater_options_, xi0_)
    , ids_to_update_()
    , timestamp_(-1)
    , is_filter_initialized_(false)
{
}

MSCEqF::MSCEqF(const std::string& params_filepath, const Quaternion& q)
    : parser_(params_filepath)
    , opts_(parser_.parseOptions())
    , X_(opts_.state_options_)
    , xi0_(opts_.state_options_)
    , track_manager_(opts_.track_manager_options_, opts_.state_options_.initial_camera_intrinsics_.k())
    , initializer_(opts_.init_options_)
    , propagator_(opts_.propagator_options_)
    , updater_(opts_.updater_options_, xi0_)
    , ids_to_update_()
    , timestamp_(-1)
    , is_filter_initialized_(false)
{
  X_.setMSCEqFStateInitialOrientation(q);
  utils::Logger::debug("Set origin to:");
  utils::Logger::debug(
      "Quaternion: " +
      static_cast<std::ostringstream&>(std::ostringstream() << xi0_.T().q().coeffs().transpose()).str());
  utils::Logger::debug("velocity: " +
                       static_cast<std::ostringstream&>(std::ostringstream() << xi0_.T().v().transpose()).str());
  utils::Logger::debug("Position: " +
                       static_cast<std::ostringstream&>(std::ostringstream() << xi0_.T().p().transpose()).str());
  utils::Logger::debug("Biases: " +
                       static_cast<std::ostringstream&>(std::ostringstream() << xi0_.b().transpose()).str());
  utils::Logger::debug(
      "Calibration euler angles (ypr): " +
      static_cast<std::ostringstream&>(std::ostringstream()
                                       << xi0_.S().q().toRotationMatrix().eulerAngles(2, 1, 0).transpose() * 180 / M_PI)
          .str());
  utils::Logger::debug("Calibration Position: " +
                       static_cast<std::ostringstream&>(std::ostringstream() << xi0_.S().x().transpose()).str());
}

void MSCEqF::processImuMeasurement(const Imu& imu)
{
  assert(imu.timestamp_ >= 0);

  // if (!is_filter_initialized_)
  // {
  //   initializer_.insertImu(imu);
  //   return;
  // }

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
    if (initializer_.detectMotion(track_manager_.tracks()))
    {
      utils::Logger::info("Static initialization succeeded");
      track_manager_.clear();
      is_filter_initialized_ = true;
    }
    // cv::imshow("Image with keypoints", cam.image_);
    // cv::waitKey(1);
    return;
  }

  if (cam.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Camera measurement older than actual state estimate. Discarding measurement");
    return;
  }

  // Parallelize propagation and image processing, synchronization is not needed since there are no data races
  auto future_propagation = std::async([&]() { return propagator_.propagate(X_, xi0_, timestamp_, cam.timestamp_); });
  auto future_image_processing = std::async([&]() { track_manager_.processCamera(cam); });

  if (!future_propagation.get())
  {
    utils::Logger::err("Propagation failure");
    return;
  }

  // Parallelize stochastic cloning and image processing, synchronization is not needed since there are no data races
  auto future_cloning = std::async([&]() { X_.stochasticCloning(cam.timestamp_); });

  future_image_processing.wait();
  future_cloning.wait();

  //
  // [VISUALIZATION-DEBUG] (Temporary)
  //
  // {
  //   Tracker::Keypoints active_kpts;
  //   std::unordered_set<uint> active_ids;
  //   track_manager_.activeTracksIds(cam.timestamp_, active_ids);
  //   const auto& tracks = track_manager_.tracks();
  //   for (auto& id : active_ids)
  //   {
  //     active_kpts.emplace_back(tracks.at(id).uvs_.back().x, tracks.at(id).uvs_.back().y, 5.0f);
  //   }
  //   cv::drawKeypoints(cam.image_, active_kpts, cam.image_, cv::Scalar(0, 0, 255),
  //                     cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  //   cv::imshow("(distorted) Image with (undistorted) keypoints", cam.image_);
  //   cv::waitKey(1);
  // }
  //
  //
  //

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
  updater_.update(X_, track_manager_.tracks(), ids_to_update_);
  if (!ids_to_update_.empty())
  {
    utils::Logger::info("Successful update with " + std::to_string(ids_to_update_.size()) + " tracks");
  }
  else
  {
    utils::Logger::warn("Failed update.");
  }

  // Update camera intrinsics if we are doing online camera intrinsic calibration
  if (X_.opts_.enable_camera_intrinsics_calibration_)
  {
    SystemState xi = stateEstimate();
    track_manager_.updateCameraIntrinsics(xi.k());
  }

  // Remove tracks used for update and clear ids_to_update_
  track_manager_.removeTracksId(ids_to_update_);
  ids_to_update_.clear();

  // Marginalize
  if (marginalize)
  {
    X_.marginalizeCloneAt(marginalize_timestamp);
    track_manager_.removeTracksTail(marginalize_timestamp);
  }
}

void MSCEqF::processFeaturesMeasurement(const TriangulatedFeatures& features)
{
  assert(features.timestamp_ >= 0);

  if (features.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Features measurement older than actual state estimate. Discarding measurement");
    return;
  }

  // Add given features to the track manager
  track_manager_.processFeatures(features);

  propagator_.propagate(X_, xi0_, timestamp_, features.timestamp_);
  X_.stochasticCloning(features.timestamp_);

  track_manager_.lostTracksIds(features.timestamp_, ids_to_update_);

  bool marginalize = false;
  fp marginalize_timestamp = -1;
  if (X_.clonesSize() == opts_.state_options_.num_clones_)
  {
    marginalize_timestamp = X_.cloneTimestampToMarginalize();
    track_manager_.activeTracksIds(marginalize_timestamp, ids_to_update_);
    marginalize = true;
  }

  updater_.update(X_, track_manager_.tracks(), ids_to_update_);
  if (!ids_to_update_.empty())
  {
    utils::Logger::info("Successful update with " + std::to_string(ids_to_update_.size()) + " tracks");
  }
  else
  {
    utils::Logger::warn("Failed update.");
  }

  if (X_.opts_.enable_camera_intrinsics_calibration_)
  {
    SystemState xi = stateEstimate();
    track_manager_.updateCameraIntrinsics(xi.k());
  }

  track_manager_.removeTracksId(ids_to_update_);
  ids_to_update_.clear();

  if (marginalize)
  {
    X_.marginalizeCloneAt(marginalize_timestamp);
    track_manager_.removeTracksTail(marginalize_timestamp);
  }
}

const MSCEqFOptions& MSCEqF::options() const { return opts_; }

const StateOptions& MSCEqF::stateOptions() const { return opts_.state_options_; }

const MatrixX& MSCEqF::Covariance() const { return X_.cov(); }

const SystemState MSCEqF::stateEstimate() const { return Symmetry::phi(X_, xi0_); }

}  // namespace msceqf