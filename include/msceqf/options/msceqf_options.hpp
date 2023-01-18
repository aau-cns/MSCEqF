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

#ifndef MSCEQF_OPTIONS_HPP
#define MSCEQF_OPTIONS_HPP

#include "types/fptypes.hpp"

namespace msceqf
{

enum class FeatureRepresentation
{
  EUCLIDEAN,
  ANCHORED_POLAR,
  ANCHORED_INVERSE_DEPTH
};

struct StateOptions
{
  /// initial covariance values
  Matrix9 D_init_cov_;      //!< Initial covariance of the D element of the state
  Matrix6 delta_init_cov_;  //!< Initial covariance of the delta element of the state
  Matrix6 E_init_cov_;      //!< Initial covariance of the E element of the state
  Matrix4 L_init_cov_;      //!< Initial covariance of the L element of the state

  /// Initial calibration values (or calibration values to be used if no online calibration is activated)
  SE3 initial_camera_extrinsic_;
  In initial_camera_intrinsic_;

  /// Filter flags
  bool enable_camera_extrinsic_calibration_ = false;  //!< Boolean to enable estrinsic camera calibration
  bool enable_camera_intrinsic_calibration_ = false;  //!< Boolean to enable intinsic camera calibration

  /// State and filter options
  fp gravity_ = 9.81;                       //!< The magnitude of the gravity vector in m/s^2
  uint num_clones_ = 10;                    //!< the maximum number of stochastic clones
  uint num_persistent_features_ = 50;       //!< the maximum number of persistent (SLAM) features
  fp persistent_feature_init_delay_ = 1.0;  //!< the delay in s before initializing persistent features
};

struct MSCEqFOptions
{
  MSCEqFOptions() = default;

  StateOptions state_options_;  //!< The state options

  // FeatureRepresentation msc_features_representation_ =
  //     FeatureRepresentation::ANCHORED_INVERSE_DEPTH;  //!< Multi State Constraint features representation
  // FeatureRepresentation persistent_features_representation_ =
  //     FeatureRepresentation::ANCHORED_INVERSE_DEPTH;  //!< persistent features representation
};

}  // namespace msceqf

#endif  // MSCEQF_OPTIONS_HPP