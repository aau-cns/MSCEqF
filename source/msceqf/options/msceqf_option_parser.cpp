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

MSCEqFOptions OptionParser::parseOptions()
{
  MSCEqFOptions opts;

  // Parse initial covariance
  parseInitialCovariance(opts.state_options_.D_init_cov_, opts.state_options_.delta_init_cov_,
                         opts.state_options_.E_init_cov_, opts.state_options_.L_init_cov_);

  // Parse camera intrinsics and extrinsics
  parseCameraCalibration(opts.state_options_.initial_camera_extrinsics_,
                         opts.state_options_.initial_camera_intrinsics_);
  readDefault(opts.state_options_.enable_camera_extrinsics_calibration_, true, "enable_camera_extrinsic_calibration");
  readDefault(opts.state_options_.enable_camera_intrinsics_calibration_, false, "enable_camera_intrinsic_calibration");

  // parse other state options
  readDefault(opts.state_options_.gravity_, fp(9.81), "gravity");
  readDefault(opts.state_options_.num_clones_, uint(10), "num_clones");
  readDefault(opts.state_options_.num_persistent_features_, uint(50), "num_persistent_features");
  readDefault(opts.state_options_.persistent_feature_init_delay_, fp(1.0), "persistent_feature_init_delay");

  return opts;
}

void OptionParser::parseCameraCalibration(SE3& extrinsics, In& intrinsics)
{
  Matrix4 extrinsics_mat;
  if (!read(extrinsics_mat, "T_cam_imu"))
  {
    throw std::runtime_error(
        "Wrong camera extrinsics provided, please provide extriniscs (T_cam_imu) in the configuration file following "
        "Kalibr convention.");
  }
  else
  {
    extrinsics = extrinsics_mat;
  }

  Vector4 intrinsics_vec;
  if (!read(intrinsics_vec, "intrinsics"))
  {
    throw std::runtime_error(
        "Wrong camera intrinsics provided, please provide intrinsics (intrinsics) in the configuration file following "
        "Kalibr convention.");
  }
  else
  {
    intrinsics = intrinsics_vec;
  }
}

void OptionParser::parseInitialCovariance(Matrix9& D_cov, Matrix6& delta_cov, Matrix6& E_cov, Matrix4& L_cov)
{
  // D covariance
  // [TODO] proper conversion prom pitch_roll_std to normal coords std
  Eigen::Matrix<fp, 2, 1> pitch_roll_std;
  if (!read(pitch_roll_std, "pitch_roll_std"))
  {
    throw std::runtime_error(
        "Wrong camera extrinsics provided, please provide extriniscs (T_cam_imu) in the configuration file following "
        "Kalibr convention.");
  }
  else
  {
    // Assign D covairiance
    // near-zero covariance for yaw and position given their unobservability
    // near-zero covariance for velocity given the static initialization
    D_cov(0) = std::pow(pitch_roll_std(0), 2);
    D_cov(1) = std::pow(pitch_roll_std(1), 2);
    D_cov.block<7, 7>(2, 2) = 1e-12 * Eigen::Matrix<fp, 7, 7>::Identity();
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

}  // namespace msceqf