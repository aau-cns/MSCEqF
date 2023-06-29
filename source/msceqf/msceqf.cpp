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
    , xi0_(opts_.state_options_)
    , X_(opts_.state_options_, xi0_)
    , track_manager_(opts_.track_manager_options_, opts_.state_options_.initial_camera_intrinsics_.k())
    , initializer_(opts_.init_options_)
    , propagator_(opts_.propagator_options_)
    , updater_(opts_.updater_options_, xi0_)
    , visualizer_(track_manager_)
    , ids_to_update_()
    , timestamp_(-1)
    , is_filter_initialized_(false)
{
  if (opts_.init_options_.init_with_given_state_)
  {
    setGivenOrigin();
  }
}

void MSCEqF::processImuMeasurement(const Imu& imu)
{
  assert(imu.timestamp_ >= 0);

  if (!is_filter_initialized_)
  {
    initializer_.insertImu(imu);
    return;
  }

  if (imu.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received IMU measurement older than actual state estimate. Discarding measurement");
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

  cam.timestamp_ += opts_.track_manager_options_.tracker_options_.cam_options_.timeshift_cam_imu_;
  opts_.track_manager_options_.tracker_options_.cam_options_.mask_.copyTo(cam.mask_);

  if (!is_filter_initialized_)
  {
    track_manager_.processCamera(cam);
    if (initializer_.detectMotion(track_manager_.tracks()))
    {
      utils::Logger::info("Static initialization succeeded");
      track_manager_.clear();

      xi0_ = SystemState(opts_.state_options_, initializer_.T0(), initializer_.b0());
      X_ = MSCEqFState(opts_.state_options_, xi0_);
      timestamp_ = cam.timestamp_;

      is_filter_initialized_ = true;
      logInit();
    }
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
  updater_.mscUpdate(X_, track_manager_.tracks(), ids_to_update_);
  if (!ids_to_update_.empty())
  {
    utils::Logger::info("Successful update with " + std::to_string(ids_to_update_.size()) + " tracks");
  }
  else
  {
    utils::Logger::warn("Failed update.");
  }

  // Update camera intrinsics if we are doing online camera intrinsic calibration
  if (X_.opts().enable_camera_intrinsics_calibration_)
  {
    SystemState xi = stateEstimate();
    track_manager_.cam()->setIntrinsics(xi.k());
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

void MSCEqF::processFeaturesMeasurement(TriangulatedFeatures& features)
{
  assert(features.timestamp_ >= 0);

  features.timestamp_ += opts_.track_manager_options_.tracker_options_.cam_options_.timeshift_cam_imu_;

  if (!is_filter_initialized_)
  {
    track_manager_.processFeatures(features);
    if (initializer_.detectMotion(track_manager_.tracks()))
    {
      utils::Logger::info("Static initialization succeeded");
      track_manager_.clear();

      if (!opts_.init_options_.init_with_given_state_)
      {
        xi0_ = SystemState(opts_.state_options_, initializer_.T0(), initializer_.b0());
        X_ = MSCEqFState(opts_.state_options_, xi0_);
      }

      timestamp_ = features.timestamp_;

      is_filter_initialized_ = true;
      logInit();
    }
    return;
  }

  if (features.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Features measurement older than actual state estimate. Discarding measurement");
    return;
  }

  propagator_.propagate(X_, xi0_, timestamp_, features.timestamp_);
  X_.stochasticCloning(features.timestamp_);

  track_manager_.processFeatures(features);
  track_manager_.lostTracksIds(features.timestamp_, ids_to_update_);

  bool marginalize = false;
  fp marginalize_timestamp = -1;
  if (X_.clonesSize() == opts_.state_options_.num_clones_)
  {
    marginalize_timestamp = X_.cloneTimestampToMarginalize();
    track_manager_.activeTracksIds(marginalize_timestamp, ids_to_update_);
    marginalize = true;
  }

  updater_.mscUpdate(X_, track_manager_.tracks(), ids_to_update_);
  if (!ids_to_update_.empty())
  {
    utils::Logger::info("Successful update with " + std::to_string(ids_to_update_.size()) + " tracks");
  }
  else
  {
    utils::Logger::warn("Failed update.");
  }

  if (X_.opts().enable_camera_intrinsics_calibration_)
  {
    SystemState xi = stateEstimate();
    track_manager_.cam()->setIntrinsics(xi.k());
  }

  track_manager_.removeTracksId(ids_to_update_);
  ids_to_update_.clear();

  if (marginalize)
  {
    X_.marginalizeCloneAt(marginalize_timestamp);
    track_manager_.removeTracksTail(marginalize_timestamp);
  }
}

void MSCEqF::setGivenOrigin()
{
  xi0_ = SystemState(
      opts_.state_options_,
      std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(
                                                    std::make_tuple(opts_.init_options_.initial_extended_pose_))),
      std::make_pair(SystemStateElementName::b,
                     createSystemStateElement<BiasState>(std::make_tuple(opts_.init_options_.initial_bias_))));

  X_ = MSCEqFState(opts_.state_options_, xi0_);
  // timestamp_ = opts_.init_options_.initial_timestamp_;

  // is_filter_initialized_ = true;
  // logInit();
}

void MSCEqF::setGivenOrigin(const SE23& T0, const Vector6& b0)
{
  xi0_ = SystemState(opts_.state_options_, T0, b0);
  X_ = MSCEqFState(opts_.state_options_, xi0_);
  timestamp_ = opts_.init_options_.initial_timestamp_;

  is_filter_initialized_ = true;
  logInit();
}

const MSCEqFOptions& MSCEqF::options() const { return opts_; }

const StateOptions& MSCEqF::stateOptions() const { return opts_.state_options_; }

const SystemState& MSCEqF::stateOrigin() const { return xi0_; }

const MatrixX& MSCEqF::covariance() const { return X_.cov(); }

const MatrixX MSCEqF::coreCovariance() const { return X_.covBlock(MSCEqFStateElementName::Dd); }

const SystemState MSCEqF::stateEstimate() const { return Symmetry::phi(X_, xi0_); }

bool MSCEqF::isInit() const { return is_filter_initialized_; }

void MSCEqF::logInit() const
{
  std::ostringstream os;

  os << "Origin set to:" << '\n'
     << "T0:" << '\n'
     << xi0_.T().asMatrix() << '\n'
     << "b0:" << '\n'
     << xi0_.b().transpose() << "\n";

  if (opts_.state_options_.enable_camera_extrinsics_calibration_)
  {
    os << "S0:" << '\n' << xi0_.S().asMatrix() << '\n';
  }
  if (opts_.state_options_.enable_camera_intrinsics_calibration_)
  {
    os << "K0:\n" << xi0_.K().asMatrix() << '\n';
  }

  os << "Set initial MSCEqF core state to" << '\n'
     << "D:" << '\n'
     << X_.D().asMatrix() << '\n'
     << "delta:" << '\n'
     << X_.delta().transpose() << '\n';

  os << "Initial time: " << timestamp_ << '\n';

  if (opts_.state_options_.enable_camera_extrinsics_calibration_)
  {
    os << "E:" << '\n' << X_.E().asMatrix() << '\n';
  }
  if (opts_.state_options_.enable_camera_intrinsics_calibration_)
  {
    os << "L:" << '\n' << X_.L().asMatrix() << '\n';
  }

  os << "Set initial MSCEqF covariance to:" << '\n' << X_.cov();

  utils::Logger::info(os.str());
}

const cv::Mat3b MSCEqF::imageWithTracks(const Camera& cam) const { return visualizer_.imageWithTracks(cam); }

void MSCEqF::visualizeImageWithTracks(const Camera& cam) const { return visualizer_.visualizeImageWithTracks(cam); }

}  // namespace msceqf