// Copyright (C) 2023 Alessandro Fornasier, Pieter van Goor.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#ifndef TEST_STATE_HPP
#define TEST_STATE_HPP

#include <iostream>

#include "msceqf/state/state.hpp"
#include "msceqf/system/system.hpp"
#include "utils/utils.hpp"

using namespace msceqf;

TEST(MSCEqFStateTest, StateConstructionTest)
{
  // Options
  MSCEqFOptions opts;

  // Enable online calibration
  opts.state_options_.enable_camera_extrinsic_calibration_ = true;
  opts.state_options_.enable_camera_intrinsic_calibration_ = true;

  // System State test
  {
    // Camera Extrinsics
    msceqf::Quaternion Sq = msceqf::Quaternion::UnitRandom();
    msceqf::Vector3 St = msceqf::Vector3::Random();
    msceqf::SE3 S = msceqf::SE3(Sq, {St});

    // Camera Intrinsics
    msceqf::Vector4 intrinsics = msceqf::Vector4::Random().cwiseAbs();
    msceqf::In K = msceqf::In(intrinsics);

    // Features
    msceqf::Vector3 feat = msceqf::Vector3::Random();
    uint feat_id = 0;

    // Default construction
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S, createSystemStateElement<CameraExtrinsicState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::K, createSystemStateElement<CameraIntrinsicState>(std::make_tuple())),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple())));

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(),
          msceqf::Matrix4::Identity());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(),
          msceqf::Matrix3::Identity());
      MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(feat_id))->f_, msceqf::Vector3::Zero());
    }

    // Parametric construction (1)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq, St))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(
                             std::make_tuple(intrinsics.x(), intrinsics.y(), intrinsics.z(), intrinsics.w()))),
          std::make_pair(feat_id,
                         createSystemStateElement<FeatureState>(std::make_tuple(feat.x(), feat.y(), feat.z()))));

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(), S.T());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(), K.K());
      MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(feat_id))->f_, feat);
    }

    // Parametric construction (2)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq.toRotationMatrix(), St))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsics))),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(), S.T());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(), K.K());
      MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(feat_id))->f_, feat);
    }

    // Parametric construction (3)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(msceqf::SO3(Sq), St))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(K.K()))),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(), S.T());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(), K.K());
      MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(feat_id))->f_, feat);
    }

    // Parametric construction (4)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S, createSystemStateElement<CameraExtrinsicState>(std::make_tuple(S))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsics))),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(), S.T());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(), K.K());
      MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(feat_id))->f_, feat);
    }

    // Parametric construction (5)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(S.T()))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsics))),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(), S.T());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(), K.K());
      MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(feat_id))->f_, feat);
    }

    // Parametric construction vector
    {
      std::vector<std::pair<SystemState::SystemStateKey, SystemStateElementUniquePtr>> feat_initializer_vector;
      for (uint i = 0; i < opts.state_options_.num_persistent_features_; ++i)
      {
        feat_initializer_vector.emplace_back(
            std::make_pair(i, createSystemStateElement<FeatureState>(std::make_tuple(feat))));
      }

      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq, St))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsics))),
          feat_initializer_vector);

      MatrixEquality(std::dynamic_pointer_cast<ExtendedPoseState>(state.state_.at(SystemStateElementName::T))->T_.T(),
                     msceqf::Matrix5::Identity());
      MatrixEquality(std::dynamic_pointer_cast<BiasState>(state.state_.at(SystemStateElementName::b))->b_,
                     msceqf::Vector6::Zero());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraExtrinsicState>(state.state_.at(SystemStateElementName::S))->S_.T(), S.T());
      MatrixEquality(
          std::dynamic_pointer_cast<CameraIntrinsicState>(state.state_.at(SystemStateElementName::K))->K_.K(), K.K());

      for (uint i = 0; i < opts.state_options_.num_persistent_features_; ++i)
      {
        MatrixEquality(std::dynamic_pointer_cast<FeatureState>(state.state_.at(i))->f_, feat);
      }
    }

    // Assertation tests
    {
      ASSERT_DEATH(
          {
            SystemState state(opts.state_options_,
                              std::make_pair(SystemStateElementName::T,
                                             createSystemStateElement<ExtendedPoseState>(std::make_tuple(Sq))));
          },
          "");
    }
  }

  // MSCEqF State test
  {
    MSCEqFState state(opts.state_options_);

    MatrixEquality(state.D().T(), msceqf::Matrix5::Identity());
    MatrixEquality(state.delta(), msceqf::Vector6::Zero());
    MatrixEquality(state.E().T(), msceqf::Matrix4::Identity());
    MatrixEquality(state.L().K(), msceqf::Matrix3::Identity());

    msceqf::Matrix15 Dd_cov = msceqf::Matrix15::Zero();
    Dd_cov.block(0, 0, 9, 9) = opts.state_options_.D_init_cov_;
    Dd_cov.block(9, 9, 6, 6) = opts.state_options_.delta_init_cov_;

    MatrixEquality(state.CovBlock(MSCEqFStateElementName::Dd), Dd_cov);
    MatrixEquality(state.CovBlock(MSCEqFStateElementName::E), opts.state_options_.E_init_cov_);
    MatrixEquality(state.CovBlock(MSCEqFStateElementName::L), opts.state_options_.L_init_cov_);
  }
}

#endif  // TEST_STATE_HPP