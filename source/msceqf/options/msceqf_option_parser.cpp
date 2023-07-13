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

  // Set the logger level
  int level;
  readDefault(level, 0, "logger_level");
  utils::Logger::setLevel(static_cast<utils::LoggerLevel>(level));

  ///
  /// Parse state options
  ///

  // Parse flags
  readDefault(opts.state_options_.enable_camera_extrinsics_calibration_, true, "enable_camera_extrinsic_calibration");
  readDefault(opts.state_options_.enable_camera_intrinsics_calibration_, false, "enable_camera_intrinsic_calibration");

  // Parse initial covariance
  parseInitialCovariance(opts.state_options_.D_init_cov_, opts.state_options_.delta_init_cov_,
                         opts.state_options_.E_init_cov_, opts.state_options_.L_init_cov_);

  // parse other state options
  readDefault(opts.state_options_.gravity_, 9.81, "gravity");
  readDefault(opts.state_options_.num_clones_, 10, "num_clones");
  readDefault(opts.state_options_.num_persistent_features_, 50, "num_persistent_features");

  ///
  /// Parse tracker parameters
  ///
  readDefault(opts.track_manager_options_.tracker_options_.max_features_, 100, "max_features");
  readDefault(opts.track_manager_options_.tracker_options_.min_features_, 20, "min_features");
  opts.track_manager_options_.tracker_options_.min_features_ =
      std::min(opts.track_manager_options_.tracker_options_.min_features_,
               opts.track_manager_options_.tracker_options_.max_features_);
  opts.track_manager_options_.tracker_options_.min_features_ =
      std::max(opts.track_manager_options_.tracker_options_.min_features_, uint(20));
  readDefault(opts.track_manager_options_.tracker_options_.grid_x_size_, 8, "grid_x_size");
  readDefault(opts.track_manager_options_.tracker_options_.grid_y_size_, 5, "grid_y_size");
  readDefault(opts.track_manager_options_.tracker_options_.min_px_dist_, 5, "min_feature_pixel_distance");
  readDefault(opts.track_manager_options_.tracker_options_.optical_flow_win_size_, 21, "optical_flow_win_size");
  readDefault(opts.track_manager_options_.tracker_options_.detector_pyramid_levels_, 3, "detector_pyramid_levels");
  readDefault(opts.track_manager_options_.tracker_options_.ransac_reprojection_, 0.5, "ransac_reprojection");
  readDefault(opts.track_manager_options_.tracker_options_.optical_flow_pyramid_levels_, 3,
              "optical_flow_pyramid_levels");

  // Parse camera parameters
  parseCameraParameters(opts.state_options_.initial_camera_extrinsics_, opts.state_options_.initial_camera_intrinsics_,
                        opts.track_manager_options_.tracker_options_.distortion_model_,
                        opts.track_manager_options_.tracker_options_.cam_options_.distortion_coefficients_,
                        opts.track_manager_options_.tracker_options_.cam_options_.resolution_,
                        opts.track_manager_options_.tracker_options_.cam_options_.timeshift_cam_imu_,
                        opts.track_manager_options_.tracker_options_.cam_options_.mask_);

  // Parse equalization method
  parseEqualizationMethod(opts.track_manager_options_.tracker_options_.equalizer_);

  // Parse feature detector type
  parseDetectorType(opts.track_manager_options_.tracker_options_.detector_);

  // Parse feature detector params
  switch (opts.track_manager_options_.tracker_options_.detector_)
  {
    case FeatureDetector::FAST:
      readDefault(opts.track_manager_options_.tracker_options_.fast_opts_.fast_threshold_, 10, "fast_threshold");
      break;
    case FeatureDetector::GFTT:
      readDefault(opts.track_manager_options_.tracker_options_.gftt_opts_.quality_level_, 0.05,
                  "shi_tomasi_quality_level");
      break;
    default:
      break;
  }

  ///
  /// Parse track manager parameters
  ///

  readDefault(opts.track_manager_options_.max_track_length_, 250, "max_track_length");

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
  /// Parse updater options
  ///

  readDefault(opts.updater_options_.refine_traingulation_, true, "refine_traingulation");
  readDefault(opts.updater_options_.min_depth_, 0.1, "feature_min_depth");
  readDefault(opts.updater_options_.max_depth_, 100, "feature_max_depth");
  readDefault(opts.updater_options_.max_iterations_, 10, "feature_refinement_max_iterations");
  readDefault(opts.updater_options_.tollerance_, 1e-12, "feature_refinement_tollerance");
  parseFeatureRepresentation(opts.updater_options_.msc_features_representation_);
  parseProjectionMethod(opts.updater_options_.projection_method_);
  readDefault(opts.updater_options_.min_track_lenght_, 5, "min_track_length");
  readDefault(opts.updater_options_.min_angle_, 0.0, "min_angle_deg");
  readDefault(opts.updater_options_.curvature_correction_, false, "curveture_correction");
  parsePixStd(opts.updater_options_.pixel_std_, opts.state_options_);

  ///
  /// Parse initalizer options
  ///

  // Initializer options
  readDefault(opts.init_options_.imu_init_window_, 0.5, "static_initializer_imu_window");
  readDefault(opts.init_options_.disparity_window_, 0.5, "static_initializer_disparity_window");
  readDefault(opts.init_options_.acc_threshold_, 0.0, "static_initializer_acc_threshold");
  readDefault(opts.init_options_.disparity_threshold_, 1.0, "static_initializer_disparity_threshold");
  readDefault(opts.init_options_.identity_b0_, false, "identity_bias_origin");
  readDefault(opts.init_options_.init_with_given_state_, false, "init_with_given_state");
  if (opts.init_options_.init_with_given_state_)
  {
    parseGivenOrigin(opts.init_options_.initial_extended_pose_, opts.init_options_.initial_bias_,
                     opts.init_options_.initial_timestamp_);
  }
  opts.init_options_.gravity_ = opts.state_options_.gravity_;

  ///
  /// Parse other options
  ///

  // Parse non state options
  // readDefault(opts.persistent_feature_init_delay_, 1.0, "persistent_feature_init_delay");

  return opts;
}

void OptionParser::parseCameraParameters(SE3& extrinsics,
                                         In& intrinsics,
                                         DistortionModel& distortion_model,
                                         VectorX& distortion_coefficients,
                                         Vector2& resolution,
                                         fp& timeshift_cam_imu,
                                         cv::Mat& mask)
{
  Matrix4 extrinsics_mat;
  if (!read(extrinsics_mat, "T_imu_cam"))
  {
    if (!read(extrinsics_mat, "T_cam_imu"))
    {
      throw std::runtime_error(
          "Wrong or missing camera extrinsics. Please provide camera extriniscs (T_imu_cam or T_cam_imu) in the "
          "configuration file according to Kalibr convention.");
    }
    else
    {
      extrinsics = extrinsics_mat;
      extrinsics = extrinsics.inv();
    }
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
    else if (model.compare("equidistant") == 0)
    {
      distortion_model = DistortionModel::EQUIDISTANT;
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

  readDefault(timeshift_cam_imu, 0.0, "timeshift_cam_imu");

  std::string maskpath;
  if (read(maskpath, "mask"))
  {
    mask = cv::imread(maskpath, cv::IMREAD_GRAYSCALE);
    cv::resize(mask, mask, cv::Size(resolution(0), resolution(1)), 0.0, 0.0, cv::INTER_NEAREST);
  }
  else
  {
    mask = 255 * cv::Mat::ones(cv::Size(resolution(0), resolution(1)), CV_8UC1);
  }
}

void OptionParser::parseGivenOrigin(SE23& T0, Vector6& b0, fp& t0)
{
  Matrix5 T = Matrix5::Identity();
  if (!read(T, "T0"))
  {
    Vector4 q;
    Vector3 p;
    Vector3 v;
    if (!read(q, "q0") & !read(p, "p0") & !read(v, "v0"))
    {
      throw std::runtime_error("Wrong or missing initial T0 or q0, v0, p0.");
    }
    else
    {
      // Note a Vector4 q is ordered as [qx, qy, qz, qw]
      T.block<3, 3>(0, 0) = SO3(Quaternion(q).normalized()).R();
      T.block<3, 1>(0, 3) = v;
      T.block<3, 1>(0, 4) = p;
    }
  }
  T0 = T;

  if (!read(b0, "b0"))
  {
    throw std::runtime_error("Wrong or missing initial bias b0.");
  }

  if (!read(t0, "t0"))
  {
    throw std::runtime_error("Wrong or missing initial timestamp t0.");
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

void OptionParser::parseFeatureRepresentation(FeatureRepresentation& rep)
{
  std::string representation;
  readDefault(representation, "anchored_inverse_depth", "feature_representation");

  if (representation.compare("anchored_euclidean") == 0)
  {
    rep = FeatureRepresentation::ANCHORED_EUCLIDEAN;
  }
  else if (representation.compare("anchored_inverse_depth") == 0)
  {
    rep = FeatureRepresentation::ANCHORED_INVERSE_DEPTH;
  }
  else if (representation.compare("anchored_polar") == 0)
  {
    rep = FeatureRepresentation::ANCHORED_POLAR;
  }
  else
  {
    throw std::runtime_error(
        "Wrong or unsupported feature representation type. Please use anchored_euclidean, anchored_inverse_depth, or "
        "anchored_polar.");
  }
}

void OptionParser::parseProjectionMethod(ProjectionMethod& proj)
{
  std::string method;
  readDefault(method, "unit_plane", "measurement_projection_method");

  if (method.compare("unit_plane") == 0)
  {
    proj = ProjectionMethod::UNIT_PLANE;
  }
  else if (method.compare("unit_sphere") == 0)
  {
    proj = ProjectionMethod::UNIT_SPHERE;
  }
  else
  {
    throw std::runtime_error(
        "Wrong or unsupported measurement projection method. Please use unit_plane or unit_sphere.");
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
  else if (detectortype.compare("shi-tomasi") == 0)
  {
    detector = FeatureDetector::GFTT;
  }
  else
  {
    throw std::runtime_error("Wrong or unsupported feature detector type. Please use fast or shi-tomasi.");
  }
}

void OptionParser::parsePixStd(fp& pix_std, const StateOptions& opts)
{
  readDefault(pix_std, 1.0, "pixel_standerd_deviation");
  if (!opts.enable_camera_intrinsics_calibration_)
  {
    pix_std /= std::max(opts.initial_camera_intrinsics_.k()(0), opts.initial_camera_intrinsics_.k()(1));
  }
}

void OptionParser::parseInitialCovariance(Matrix9& D_cov, Matrix6& delta_cov, Matrix6& E_cov, Matrix4& L_cov)
{
  // D covariance
  Vector9 extended_pose_std;
  if (!read(extended_pose_std, "extended_pose_std"))
  {
    throw std::runtime_error(
        "Wrong or missing pitch and roll standard deviation. Please provide attitude, velocity, and position standard "
        "deviation as extended_pose_std parameter.");
  }
  else
  {
    // Assign D covairiance
    D_cov = extended_pose_std.array().square().matrix().asDiagonal();
  }

  // Delta covariance
  Vector6 bias_std;
  if (!read(bias_std, "bias_std"))
  {
    throw std::runtime_error(
        "Wrong or missing bias standard deviation. Please provide bias standard deviation as bias_std parameter.");
  }
  else
  {
    delta_cov = bias_std.array().square().matrix().asDiagonal();
  }

  // E covariance
  Vector6 extrinsics_std;
  if (!read(extrinsics_std, "extrinsics_std"))
  {
    utils::Logger::warn(
        "Extrinsics initial standard deviation not provided. Set to default value. Used only if online extrinsics "
        "calibration is enabled");
    E_cov = 1e-4 * Matrix6::Identity();
  }
  else
  {
    E_cov = extrinsics_std.array().square().matrix().asDiagonal();
  }

  // L covairance
  Vector4 intrinsics_std;
  if (!read(intrinsics_std, "intrinsics_std"))
  {
    utils::Logger::warn(
        "Intrinsics initial standard deviation not provided. Set to default value. Used only if online intrinsics "
        "calibration is enbled.");
    L_cov = 10 * Matrix4::Identity();
  }
  else
  {
    L_cov = intrinsics_std.array().square().matrix().asDiagonal();
  }
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