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
    , xi_(opts_.state_options_)
    , track_manager_(opts_.track_manager_options_, opts_.state_options_.initial_camera_intrinsics_.k())
    , checker_(opts_.checker_options_)
    , initializer_(opts_.init_options_, checker_)
    , propagator_(opts_.propagator_options_)
    , updater_(opts_.updater_options_, xi0_)
    , zvupdater_(opts_.zvupdater_options_, checker_)
    , visualizer_(track_manager_)
    , ids_to_update_()
    , timestamp_(-1)
    , is_filter_initialized_(false)
    , zvu_performed_(false)
{
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
  assert(cam.image_.size() == cv::Size(opts_.track_manager_options_.tracker_options_.cam_options_.resolution_(0),
                                       opts_.track_manager_options_.tracker_options_.cam_options_.resolution_(1)));

  cam.timestamp_ += opts_.track_manager_options_.tracker_options_.cam_options_.timeshift_cam_imu_;

  if (opts_.track_manager_options_.tracker_options_.cam_options_.mask_type_ == MaskType::STATIC)
  {
    assert(cam.mask_.size() == opts_.track_manager_options_.tracker_options_.cam_options_.static_mask_.size());
    cam.mask_ = opts_.track_manager_options_.tracker_options_.cam_options_.static_mask_;
  }

  if (!is_filter_initialized_)
  {
    initialize(cam);
    return;
  }

  if (cam.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Camera measurement older than actual state estimate. Discarding measurement");
    return;
  }

  auto future_propagation = std::async([&]() { return propagator_.propagate(X_, xi0_, timestamp_, cam.timestamp_); });
  auto future_image_processing = std::async([&]() { track_manager_.processCamera(cam); });

  if (!future_propagation.get())
  {
    utils::Logger::err("Propagation failure");
    return;
  }

  if (opts_.zvupdater_options_.zero_velocity_update_ != ZeroVelocityUpdate::DISABLE)
  {
    future_image_processing.wait();
    if (zvupdater_.isActive(track_manager_.tracks()))
    {
      if (!zvu_performed_)
      {
        zvupdater_.setMeasurement(SE23(SO3(), {-xi_.T().v(), Vector3::Zero()}).multiplyRight(xi_.T()));
      }
      zvu_performed_ = zvupdater_.zvUpdate(X_, xi0_);
      utils::Logger::info("Successful zero velocity update");
      return;
    }
    if (zvu_performed_)
    {
      zvu_performed_ = false;
      track_manager_.removeTracksTail(cam.timestamp_, false);
    }
    X_.stochasticCloning(cam.timestamp_);
  }
  else
  {
    auto future_cloning = std::async([&]() { X_.stochasticCloning(cam.timestamp_); });

    future_image_processing.wait();
    future_cloning.wait();
  }

  track_manager_.lostTracksIds(cam.timestamp_, ids_to_update_);

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

  xi_ = Symmetry::phi(X_, xi0_);

  if (X_.opts().enable_camera_intrinsics_calibration_)
  {
    track_manager_.cam()->setIntrinsics(xi_.k());
  }

  track_manager_.removeTracksId(ids_to_update_);
  ids_to_update_.clear();

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
    initialize(features);
    return;
  }

  if (features.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Features measurement older than actual state estimate. Discarding measurement");
    return;
  }

  auto future_propagation =
      std::async([&]() { return propagator_.propagate(X_, xi0_, timestamp_, features.timestamp_); });
  auto future_feature_processing = std::async([&]() { track_manager_.processFeatures(features); });

  if (!future_propagation.get())
  {
    utils::Logger::err("Propagation failure");
    return;
  }

  if (opts_.zvupdater_options_.zero_velocity_update_ != ZeroVelocityUpdate::DISABLE)
  {
    future_feature_processing.wait();
    if (zvupdater_.isActive(track_manager_.tracks()))
    {
      if (!zvu_performed_)
      {
        zvupdater_.setMeasurement(SE23(SO3(), {-xi_.T().v(), Vector3::Zero()}).multiplyRight(xi_.T()));
      }
      zvu_performed_ = zvupdater_.zvUpdate(X_, xi0_);
      utils::Logger::info("Successful zero velocity update");
      return;
    }
    if (zvu_performed_)
    {
      zvu_performed_ = false;
      track_manager_.removeTracksTail(features.timestamp_, false);
    }
    X_.stochasticCloning(features.timestamp_);
  }
  else
  {
    auto future_cloning = std::async([&]() { X_.stochasticCloning(features.timestamp_); });

    future_feature_processing.wait();
    future_cloning.wait();
  }

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

  xi_ = Symmetry::phi(X_, xi0_);

  if (X_.opts().enable_camera_intrinsics_calibration_)
  {
    track_manager_.cam()->setIntrinsics(xi_.k());
  }

  track_manager_.removeTracksId(ids_to_update_);
  ids_to_update_.clear();

  if (marginalize)
  {
    X_.marginalizeCloneAt(marginalize_timestamp);
    track_manager_.removeTracksTail(marginalize_timestamp);
  }
}

void MSCEqF::initialize(Camera& cam)
{
  if (opts_.init_options_.init_with_given_state_)
  {
    setGivenOrigin();
    return;
  }

  track_manager_.processCamera(cam);
  if (opts_.zvupdater_options_.zero_velocity_update_ != ZeroVelocityUpdate::DISABLE)
  {
    if (initializer_.initializeOrigin())
    {
      setGivenOrigin(initializer_.T0(), initializer_.b0());
    }
  }
  else
  {
    if (initializer_.detectMotion(track_manager_.tracks()))
    {
      utils::Logger::info("Static initialization succeeded");
      track_manager_.clear();
      setGivenOrigin(initializer_.T0(), initializer_.b0());
    }
  }
  return;
}

void MSCEqF::initialize(TriangulatedFeatures& features)
{
  if (opts_.init_options_.init_with_given_state_)
  {
    setGivenOrigin();
    return;
  }

  track_manager_.processFeatures(features);
  if (opts_.zvupdater_options_.zero_velocity_update_ != ZeroVelocityUpdate::DISABLE)
  {
    if (initializer_.initializeOrigin())
    {
      setGivenOrigin(initializer_.T0(), initializer_.b0());
    }
  }
  else
  {
    if (initializer_.detectMotion(track_manager_.tracks()))
    {
      utils::Logger::info("Static initialization succeeded");
      track_manager_.clear();
      setGivenOrigin(initializer_.T0(), initializer_.b0());
    }
  }
  return;
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
  xi_ = Symmetry::phi(X_, xi0_);
  timestamp_ = opts_.init_options_.initial_timestamp_;

  is_filter_initialized_ = true;
  logInit();
}

void MSCEqF::setGivenOrigin(const SE23& T0, const Vector6& b0)
{
  xi0_ = SystemState(opts_.state_options_, T0, b0);
  X_ = MSCEqFState(opts_.state_options_, xi0_);
  xi_ = Symmetry::phi(X_, xi0_);
  timestamp_ = opts_.init_options_.initial_timestamp_;

  is_filter_initialized_ = true;
  logInit();
}

const MSCEqFOptions& MSCEqF::options() const { return opts_; }

const StateOptions& MSCEqF::stateOptions() const { return opts_.state_options_; }

const SystemState& MSCEqF::stateOrigin() const { return xi0_; }

const MatrixX& MSCEqF::covariance() const { return X_.cov(); }

const MatrixX MSCEqF::coreCovariance() const { return X_.covBlock(MSCEqFStateElementName::Dd); }

const SystemState& MSCEqF::stateEstimate() const { return xi_; }

const bool& MSCEqF::isInit() const { return is_filter_initialized_; }

const bool& MSCEqF::zvuPerformed() const { return zvu_performed_; }

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

const cv::Mat3b MSCEqF::imageWithTracks(const Camera& cam) const
{
  std::string text = "";
  if (!is_filter_initialized_)
  {
    text = "Initialization";
  }
  else if (zvu_performed_)
  {
    text = "ZeroVelocityUpdate";
  }
  return visualizer_.imageWithTracks(cam, text);
}

void MSCEqF::visualizeImageWithTracks(const Camera& cam) const
{
  std::string text = "";
  if (!is_filter_initialized_)
  {
    text = "Initialization";
  }
  else if (zvu_performed_)
  {
    text = "ZeroVelocityUpdate";
  }
  return visualizer_.visualizeImageWithTracks(cam, text);
}

}  // namespace msceqf