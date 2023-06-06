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

#ifndef OPTIONS_HPP
#define OPTIONS_HPP

#include "types/fptypes.hpp"

namespace msceqf
{
enum class FeatureRepresentation
{
  ANCHORED_EUCLIDEAN,
  ANCHORED_POLAR,
  ANCHORED_INVERSE_DEPTH,
};

enum class ProjectionMethod
{
  UNIT_SPHERE,
  UNIT_PLANE,
};

enum class DistortionModel
{
  RADTAN,
  EQUIDISTANT,
};

enum class EqualizationMethod
{
  HISTOGRAM,
  CLAHE,
  NONE,
};

enum class FeatureDetector
{
  FAST,
  GFTT,
};

/// @note The camera extrinsics are interpreted as IC_S, thus IC_S transofrm vectors in camera frame to vectors in imu
/// frame according to the following equation: I_x = IC_S * C_x
struct StateOptions
{
  Matrix9 D_init_cov_;                         //!< Initial covariance of the D element of the state
  Matrix6 delta_init_cov_;                     //!< Initial covariance of the delta element of the state
  Matrix6 E_init_cov_;                         //!< Initial covariance of the E element of the state
  Matrix4 L_init_cov_;                         //!< Initial covariance of the L element of the state
  SE3 initial_camera_extrinsics_;              //!< Initial camera extrinsics
  In initial_camera_intrinsics_;               //!< Initial camera intrinsics
  bool enable_camera_extrinsics_calibration_;  //!< Boolean to enable estrinsic camera calibration
  bool enable_camera_intrinsics_calibration_;  //!< Boolean to enable intinsic camera calibration
  fp gravity_;                                 //!< The magnitude of the gravity vector in m/s^2
  uint num_clones_;                            //!< The maximum number of stochastic clones
  uint num_persistent_features_;               //!< The maximum number of persistent (SLAM) features
};

struct PropagatorOptions
{
  fp angular_velocity_std_;       //!< Continuous time angular velocity standard deviation
  fp acceleration_std_;           //!< Continuous time acceleration standard deviation
  fp angular_velocity_bias_std_;  //!< Continuous time angular velocity bias (random walk) standard deviation
  fp acceleration_bias_std_;      //!< Continuous time acceleration bias (random walk) standard deviation
  uint imu_buffer_max_size_;      //!< The maximum size of the propagator's imu buffer
  int state_transition_order_;    //!< The order for the computation of the state transition matrix
};

struct UpdaterOptions
{
  bool refine_traingulation_;  //!< Boolean to enable feature triangulation refinement via nonlinear optimization
  fp min_depth_;               //!< Minimum depth of triangulated features
  fp max_depth_;               //!< Maximum depth of triangulated features
  uint max_iterations_;        //!< Maximum number of iteration for features triangulation refinement
  fp tollerance_;              //!< Tollerance for features triangulation refinement
  FeatureRepresentation msc_features_representation_;  //!< Multi State Constraint features representation
  ProjectionMethod projection_method_;                 //!< The feature projection method
  uint min_track_lenght_;                              //!< Minimum track length for triangulation
  fp min_angle_;                                       //!< Minimum angle (in degrees) between views for trianglulation
  fp pixel_std_;                                       //!< The pixel standard deviation
  bool curvature_correction_;                          //!< Boolean to enable the curvature correction
};

struct InitializerOptions
{
  fp disparity_threshold_;      //!< the disparity threshold for the static initializer
  fp acc_threshold_;            //!< The acceleration threshold for the static initializer
  fp imu_init_window_;          //!< The window in seconds used to check for acceleration spikes
  fp disparity_window_;         //!< The window is seconds used to check disparity
  fp gravity_;                  //!< The magnitude of the gravity vector in m/s^2
  bool identity_b0_;            //!< Boolean to fix identity bias origin (b0)git
  bool init_with_given_state_;  //!< Boolean to initialize the state with the given state
  SE23 initial_extended_pose_;  //!< Initial extended pose
  Vector6 initial_bias_;        //!< Initial bias
  fp initial_timestamp_;        //!< Initial timestamp
};

struct CameraOptions
{
  VectorX distortion_coefficients_;  //!< Distortion coefficients
  Vector2 resolution_;               //!< Width, Height
  fp timeshift_cam_imu_;             //!< The time shift between camera and imu (t_imu = t_cam + shift)
};

struct FastOptions
{
  int fast_threshold_;  //!< Fast detector threshold (The lower the more feature are detected/accepted)
};

struct GFTTOptions
{
  fp quality_level_;  //!< Shi-Tomasi detector quality level (The lower the more feature are detected/accepted)
};

struct TrackerOptions
{
  CameraOptions cam_options_;         //!< The camera options
  DistortionModel distortion_model_;  //!< Distortion Model
  EqualizationMethod equalizer_;      //!< The image equalization method
  FeatureDetector detector_;          //!< The feature detector
  uint max_features_;                 //!< maximum feature to track/detect
  uint min_features_;                 //!< Minimum feature to track/detect
  uint grid_x_size_;                  //!< x size of the grid
  uint grid_y_size_;                  //!< y size of the grid
  uint min_px_dist_;                  //!< minimum pixel distance between features
  uint optical_flow_pyramid_levels_;  //!< pyramids levels for optical flow (1-based)
  uint detector_pyramid_levels_;      //!< pyramids levels for feature detection (1-based)
  uint optical_flow_win_size_;        //!< window size for optical flow
  fp ransac_reprojection_;            //!< RANSAC reprojection threshold
  FastOptions fast_opts_;             //!< Fast feature detector options
  GFTTOptions gftt_opts_;             //!< Shi-Tomasi feature detector options
};

struct TrackManagerOptions
{
  TrackerOptions tracker_options_;  //!< The vision tracker options
  size_t max_track_length_;         //!< The maximul length of a track
};

struct MSCEqFOptions
{
  StateOptions state_options_;                 //!< The state options
  PropagatorOptions propagator_options_;       //!< The propagator options
  UpdaterOptions updater_options_;             //!< The updater options
  InitializerOptions init_options_;            //!< The initializer options
  TrackManagerOptions track_manager_options_;  //!< The track manager options
};

}  // namespace msceqf

#endif  // OPTIONS_HPP