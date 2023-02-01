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

#include "msceqf/options/msceqf_option_parser.hpp"

#include <exception>
#include <sstream>

namespace msceqf
{
OptionParser::OptionParser(const std::string& filepath) : node_(YAML::LoadFile(filepath)), filepath_(filepath) {}

// [TODO] Check that every option is covered
MSCEqFOptions OptionParser::parseOptions()
{
  MSCEqFOptions opts;

  ///
  /// Parse state options
  ///

  // Parse initial covariance
  parseInitialCovariance(opts.state_options_.D_init_cov_, opts.state_options_.delta_init_cov_,
                         opts.state_options_.E_init_cov_, opts.state_options_.L_init_cov_);

  // Parse flags
  readDefault(opts.state_options_.enable_camera_extrinsics_calibration_, true, "enable_camera_extrinsic_calibration");
  readDefault(opts.state_options_.enable_camera_intrinsics_calibration_, false, "enable_camera_intrinsic_calibration");

  // parse other state options
  readDefault(opts.state_options_.gravity_, 9.81, "gravity");
  readDefault(opts.state_options_.num_clones_, 10, "num_clones");
  readDefault(opts.state_options_.num_persistent_features_, 50, "num_persistent_features");

  ///
  /// Parse tracker parameters
  ///
  readDefault(opts.tracker_options_.max_features_, 100, "max_features");
  uint min_feats_default = std::min(uint(20), opts.tracker_options_.max_features_);
  readDefault(opts.tracker_options_.min_features_, min_feats_default, "min_features");
  readDefault(opts.tracker_options_.grid_x_size_, 8, "grid_x_size");
  readDefault(opts.tracker_options_.grid_y_size_, 5, "grid_y_size");
  readDefault(opts.tracker_options_.min_px_dist_, 5, "min_feature_pixel_distance");

  readDefault(opts.tracker_options_.pyramid_levels_, 3, "optical_flow_pyramid_levels");
  readDefault(opts.tracker_options_.optical_flow_win_size_, 21, "optical_flow_win_size");

  // Parse camera parameters
  parseCameraParameters(opts.state_options_.initial_camera_extrinsics_, opts.state_options_.initial_camera_intrinsics_,
                        opts.tracker_options_.distortion_model_,
                        opts.tracker_options_.cam_options_.distortion_coefficients_,
                        opts.tracker_options_.cam_options_.resolution_);

  // Parse equalization method
  parseEqualizationMethod(opts.tracker_options_.equalizer_);

  // Parse feature detector type
  parseDetectorType(opts.tracker_options_.detector_);

  // Parse feature detector params
  switch (opts.tracker_options_.detector_)
  {
    case FeatureDetector::FAST:
      readDefault(opts.tracker_options_.fast_opts_.fast_threshold_, 10, "fast_threshold");
      break;
    case FeatureDetector::GFTT:
      readDefault(opts.tracker_options_.gftt_opts_.quality_level_, 0.05, "shi_tomasi_quality_level");
      break;
    default:
      break;
  }

  ///
  /// Parse propagator options
  ///

  // Parse process noise covariance
  parseProcessNoise(opts.propagator_options_.angular_velocity_std_, opts.propagator_options_.acceleration_std_,
                    opts.propagator_options_.angular_velocity_bias_std_,
                    opts.propagator_options_.acceleration_bias_std_);

  // parse other propagator options
  readDefault(opts.propagator_options_.state_transition_order_, 1, "state_transition_order");
  readDefault(opts.propagator_options_.imu_buffer_max_size_, 1000, "imu_buffer_max_size");

  ///
  /// Parse initalizer options
  ///

  // Initializer options
  readDefault(opts.init_options_.imu_init_window_, 0.5, "static_initializer_imu_window");
  readDefault(opts.init_options_.disparity_threshold_, 1.0, "static_initializer_disparity_threshold");
  readDefault(opts.init_options_.acc_threshold_, 0.0, "static_initializer_acc_threshold");

  ///
  /// Parse other options
  ///

  // Parse non state options
  // readDefault(opts.persistent_feature_init_delay_, 1.0, "persistent_feature_init_delay");

  // Set the logger level
  int level;
  readDefault(level, 0, "logger_level");
  utils::Logger::setLevel(static_cast<utils::LoggerLevel>(level));

  return opts;
}

void OptionParser::parseCameraParameters(SE3& extrinsics,
                                         In& intrinsics,
                                         DistortionModel& distortion_model,
                                         VectorX& distortion_coefficients,
                                         Vector2& resolution)
{
  Matrix4 extrinsics_mat;
  if (!read(extrinsics_mat, "T_cam_imu"))
  {
    throw std::runtime_error(
        "Wrong or missing camera extrinsics. Please provide camera extriniscs (T_cam_imu) in the configuration file "
        "according to Kalibr convention.");
  }
  else
  {
    extrinsics = extrinsics_mat;
  }

  Vector4 intrinsics_vec;
  if (!read(intrinsics_vec, "intrinsics"))
  {
    throw std::runtime_error(
        "Wrong or missing camera intrinsics. Please provide camera intrinsics (intrinsics) in the configuration "
        "file according to Kalibr convention.");
  }
  else
  {
    intrinsics = intrinsics_vec;
  }

  if (!read(distortion_coefficients, "distortion_coeffs"))
  {
    throw std::runtime_error(
        "Wrong or missing camera distortion coefficients. Please provide camera distortion coefficients "
        "(distortion_coeffs) in the configuration file according to Kalibr convention.");
  }

  std::string model;
  if (!read(model, "distortion_model"))
  {
    throw std::runtime_error(
        "Wrong or missing camera distortion model. Please provide camera distortion model (distortion_model) "
        "in the configuration file according to Kalibr convention.");
  }
  else
  {
    if (model.compare("radtan") == 0)
    {
      distortion_model = DistortionModel::RADTAN;
    }
    else
    {
      throw std::runtime_error(model + " distortion model not supported.");
    }
  }

  if (!read(resolution, "resolution"))
  {
    throw std::runtime_error(
        "Wrong or missing camera resolution. Please provide camera resolution (resolution) in the configuration "
        "file according to Kalibr convention.");
  }
}

void OptionParser::parseEqualizationMethod(EqualizationMethod& eq)
{
  std::string method;
  readDefault(method, "none", "equalization_method");

  if (method.compare("none") == 0)
  {
    eq = EqualizationMethod::NONE;
  }
  else if (method.compare("histogram") == 0)
  {
    eq = EqualizationMethod::HISTOGRAM;
  }
  else if (method.compare("clahe") == 0)
  {
    eq = EqualizationMethod::CLAHE;
  }
  else
  {
    throw std::runtime_error("Wrong or unsupported equalization method. Please use histogram or clahe.");
  }
}

void OptionParser::parseDetectorType(FeatureDetector& detector)
{
  std::string detectortype;
  readDefault(detectortype, "fast", "feature_detector");

  if (detectortype.compare("fast") == 0)
  {
    detector = FeatureDetector::FAST;
  }
  else if (detectortype.compare("shi_tomasi") == 0)
  {
    detector = FeatureDetector::GFTT;
  }
  else
  {
    throw std::runtime_error("Wrong or unsupported feature detector type. Please use fast or shi-tomasi.");
  }
}

void OptionParser::parseInitialCovariance(Matrix9& D_cov, Matrix6& delta_cov, Matrix6& E_cov, Matrix4& L_cov)
{
  // D covariance
  // [TODO] proper conversion prom pitch_roll_std to normal coords std
  Vector2 pitch_roll_std;
  if (!read(pitch_roll_std, "pitch_roll_std"))
  {
    throw std::runtime_error(
        "Wrong or missing pitch and roll standard deviation. Please provide pitch and roll standard deviation as "
        "pitch_roll_std parameter.");
  }
  else
  {
    // Assign D covairiance
    // near-zero covariance for yaw and position given their unobservability
    // near-zero covariance for velocity given the static initialization
    D_cov = Matrix9::Zero();
    D_cov(0, 0) = std::pow(pitch_roll_std(0), 2);
    D_cov(1, 1) = std::pow(pitch_roll_std(1), 2);
    D_cov.block<7, 7>(2, 2) = 1e-12 * Matrix7::Identity();
  }

  // Delta covariance
  // [TODO] proper conversion prom pitch_roll_std to normal coords std
  delta_cov = 1e-6 * Matrix6::Identity();

  // E covariance
  // [TODO] Assign proper values
  // [TODO] proper conversion prom pitch_roll_std to normal coords std
  E_cov = 0.1 * Matrix6::Identity();

  // L covairance
  // [TODO] Assign proper values
  // [TODO] proper conversion prom pitch_roll_std to normal coords std
  L_cov = Matrix4::Identity();
}

void OptionParser::parseProcessNoise(fp& w_std, fp& a_std, fp& bw_std, fp& ba_std)
{
  if (!read(w_std, "gyroscope_noise_density"))
  {
    throw std::runtime_error(
        "Wrong or missing gyroscope noise density. Please provide gyroscope noise density "
        "(gyroscope_noise_density) in the configuration file according to Kalibr convention.");
  }
  if (!read(a_std, "accelerometer_noise_density"))
  {
    throw std::runtime_error(
        "Wrong or missing accelerometer noise density. Please provide accelerometer noise density "
        "(accelerometer_noise_density) in the configuration file according to Kalibr convention.");
  }
  if (!read(bw_std, "gyroscope_random_walk"))
  {
    throw std::runtime_error(
        "Wrong or missing gyroscope random walk. Please provide gyroscope random walk "
        "(gyroscope_random_walk) in the configuration file according to Kalibr convention.");
  }
  if (!read(ba_std, "accelerometer_random_walk"))
  {
    throw std::runtime_error(
        "Wrong or missing accelerometer random walk. Please provide accelerometer random walk "
        "(accelerometer_random_walk) in the configuration file according to Kalibr convention.");
  }
}

}  // namespace msceqf