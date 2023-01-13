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

#ifndef TEST_STATE_HPP
#define TEST_STATE_HPP

#include <iostream>

#include "msceqf/state/state.hpp"
#include "msceqf/system/system.hpp"

namespace msceqf
{

TEST(SystemStateTest, SystemStateConstructionTest)
{
  // Options
  MSCEqFOptions opts;

  // Enable online calibration
  opts.state_options_.enable_camera_extrinsic_calibration_ = true;
  opts.state_options_.enable_camera_intrinsic_calibration_ = true;

  for (int i = 0; i < N_TESTS; ++i)
  {
    // Camera Extrinsics
    Quaternion Sq = Quaternion::UnitRandom();
    Vector3 St = Vector3::Random();
    Matrix4 S = SE3(Sq, {St}).asMatrix();

    // Camera Intrinsics
    Vector4 intrinsics = Vector4::Random().cwiseAbs();
    Matrix3 K = In(intrinsics).asMatrix();

    // Features
    Vector3 feat = Vector3::Random();
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

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), Matrix4::Identity());
      MatrixEquality(state.K().asMatrix(), Matrix3::Identity());
      MatrixEquality(state.f(feat_id), Vector3::Zero());
    }

    // Parametric construction (1)
    {
      SystemState state(opts.state_options_,
                        std::make_pair(SystemStateElementName::T,
                                       createSystemStateElement<ExtendedPoseState>(std::make_tuple(SE23()))),
                        std::make_pair(SystemStateElementName::b,
                                       createSystemStateElement<BiasState>(std::make_tuple(Vector6::Zero()))),
                        std::make_pair(SystemStateElementName::S,
                                       createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq, St))),
                        std::make_pair(SystemStateElementName::K,
                                       createSystemStateElement<CameraIntrinsicState>(std::make_tuple(
                                           intrinsics.x(), intrinsics.y(), intrinsics.z(), intrinsics.w()))),
                        std::make_pair(feat_id, createSystemStateElement<FeatureState>(
                                                    std::make_tuple(feat.x(), feat.y(), feat.z()))));

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), S);
      MatrixEquality(state.K().asMatrix(), K);
      MatrixEquality(state.f(feat_id), feat);
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

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), S);
      MatrixEquality(state.K().asMatrix(), K);
      MatrixEquality(state.f(feat_id), feat);
    }

    // Parametric construction (3)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S, createSystemStateElement<CameraExtrinsicState>(std::make_tuple(S))),
          std::make_pair(SystemStateElementName::K, createSystemStateElement<CameraIntrinsicState>(std::make_tuple(K))),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), S);
      MatrixEquality(state.K().asMatrix(), K);
      MatrixEquality(state.f(feat_id), feat);
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

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), S);
      MatrixEquality(state.K().asMatrix(), K);
      for (uint i = 0; i < opts.state_options_.num_persistent_features_; ++i)
      {
        MatrixEquality(state.f(i), feat);
      }
    }

    // Copy and assignment
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

      SystemState state_copy(state);

      MatrixEquality(state.T().asMatrix(), state_copy.T().asMatrix());
      MatrixEquality(state.b(), state_copy.b());
      MatrixEquality(state.S().asMatrix(), state_copy.S().asMatrix());
      MatrixEquality(state.K().asMatrix(), state_copy.K().asMatrix());
      for (uint i = 0; i < opts.state_options_.num_persistent_features_; ++i)
      {
        MatrixEquality(state.f(i), state_copy.f(i));
      }

      SystemState state_copy_2 = state;

      MatrixEquality(state.T().asMatrix(), state_copy_2.T().asMatrix());
      MatrixEquality(state.b(), state_copy_2.b());
      MatrixEquality(state.S().asMatrix(), state_copy_2.S().asMatrix());
      MatrixEquality(state.K().asMatrix(), state_copy_2.K().asMatrix());
      for (uint i = 0; i < opts.state_options_.num_persistent_features_; ++i)
      {
        MatrixEquality(state.f(i), state_copy_2.f(i));
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
}

TEST(MSCEqFStateTest, MSCEqFStateConstructionTest)
{
  // Options
  MSCEqFOptions opts;

  // Enable online calibration
  opts.state_options_.enable_camera_extrinsic_calibration_ = true;
  opts.state_options_.enable_camera_intrinsic_calibration_ = true;

  for (int i = 0; i < N_TESTS; ++i)
  {
    // Construction, copy, assignment
    {
      MSCEqFState state(opts.state_options_);

      MatrixEquality(state.D().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.delta(), Vector6::Zero());
      MatrixEquality(state.E().asMatrix(), Matrix4::Identity());
      MatrixEquality(state.L().asMatrix(), Matrix3::Identity());

      Matrix15 Dd_cov = Matrix15::Zero();
      Dd_cov.block(0, 0, 9, 9) = opts.state_options_.D_init_cov_;
      Dd_cov.block(9, 9, 6, 6) = opts.state_options_.delta_init_cov_;

      MatrixEquality(state.CovBlock(MSCEqFStateElementName::Dd), Dd_cov);
      MatrixEquality(state.CovBlock(MSCEqFStateElementName::E), opts.state_options_.E_init_cov_);
      MatrixEquality(state.CovBlock(MSCEqFStateElementName::L), opts.state_options_.L_init_cov_);

      MSCEqFState state_copy(state);

      MatrixEquality(state_copy.D().asMatrix(), state.D().asMatrix());
      MatrixEquality(state_copy.delta(), state.delta());
      MatrixEquality(state_copy.E().asMatrix(), state.E().asMatrix());
      MatrixEquality(state_copy.L().asMatrix(), state.L().asMatrix());
      MatrixEquality(state_copy.CovBlock(MSCEqFStateElementName::Dd), state.CovBlock(MSCEqFStateElementName::Dd));
      MatrixEquality(state_copy.CovBlock(MSCEqFStateElementName::E), state.CovBlock(MSCEqFStateElementName::E));
      MatrixEquality(state_copy.CovBlock(MSCEqFStateElementName::L), state.CovBlock(MSCEqFStateElementName::L));

      MSCEqFState state_copy_2 = state;

      MatrixEquality(state_copy_2.D().asMatrix(), state.D().asMatrix());
      MatrixEquality(state_copy_2.delta(), state.delta());
      MatrixEquality(state_copy_2.E().asMatrix(), state.E().asMatrix());
      MatrixEquality(state_copy_2.L().asMatrix(), state.L().asMatrix());
      MatrixEquality(state_copy_2.CovBlock(MSCEqFStateElementName::Dd), state.CovBlock(MSCEqFStateElementName::Dd));
      MatrixEquality(state_copy_2.CovBlock(MSCEqFStateElementName::E), state.CovBlock(MSCEqFStateElementName::E));
      MatrixEquality(state_copy_2.CovBlock(MSCEqFStateElementName::L), state.CovBlock(MSCEqFStateElementName::L));
    }
  }
}

}  // namespace msceqf

#endif  // TEST_STATE_HPP