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
#include "msceqf/symmetry/symmetry.hpp"
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
    // Camera Extrinsic
    Quaternion Sq = Quaternion::UnitRandom();
    Vector3 St = Vector3::Random();

    opts.state_options_.initial_camera_extrinsic_ = SE3(Sq, {St});
    Matrix4 S = opts.state_options_.initial_camera_extrinsic_.asMatrix();

    // Camera Intrinsic
    Vector4 intrinsic = Vector4::Random().cwiseAbs();

    opts.state_options_.initial_camera_intrinsic_ = In(intrinsic);
    Matrix3 K = opts.state_options_.initial_camera_intrinsic_.asMatrix();

    // Feature
    Vector3 feat = Vector3::Random();
    uint feat_id = utils::random<int>(0, 1000);

    // Features ids
    std::vector<uint> feat_ids;
    while (feat_ids.size() != opts.state_options_.num_persistent_features_)
    {
      uint id = utils::random<int>(0, 1000);
      if (std::find(feat_ids.begin(), feat_ids.end(), id) == feat_ids.end())
      {
        feat_ids.emplace_back(id);
      }
    }

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

    // Default construction without persistent features
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S, createSystemStateElement<CameraExtrinsicState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::K, createSystemStateElement<CameraIntrinsicState>(std::make_tuple())));

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), Matrix4::Identity());
      MatrixEquality(state.K().asMatrix(), Matrix3::Identity());
    }

    // Default construction without camera intrinsic, extrinsic, and persistent features
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(feat_id, createSystemStateElement<FeatureState>(std::make_tuple())));

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.f(feat_id), Vector3::Zero());
    }

    // Default construction without camera intrinsic, extrinsic, but with persistent features
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())));

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
    }

    // Implicit extrinsic and intrinsic construction
    {
      SystemState state(opts.state_options_,
                        std::make_pair(SystemStateElementName::T,
                                       createSystemStateElement<ExtendedPoseState>(std::make_tuple(SE23()))),
                        std::make_pair(SystemStateElementName::b,
                                       createSystemStateElement<BiasState>(std::make_tuple(Vector6::Zero()))),
                        std::make_pair(feat_id, createSystemStateElement<FeatureState>(
                                                    std::make_tuple(feat.x(), feat.y(), feat.z()))));

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), S);
      MatrixEquality(state.K().asMatrix(), K);
      MatrixEquality(state.f(feat_id), feat);
    }

    // Parametric construction (1)
    {
      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T,
                         createSystemStateElement<ExtendedPoseState>(std::make_tuple(SE23()))),
          std::make_pair(SystemStateElementName::b,
                         createSystemStateElement<BiasState>(std::make_tuple(Vector6::Zero()))),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq, St))),
          std::make_pair(SystemStateElementName::K, createSystemStateElement<CameraIntrinsicState>(std::make_tuple(
                                                        intrinsic.x(), intrinsic.y(), intrinsic.z(), intrinsic.w()))),
          std::make_pair(feat_id,
                         createSystemStateElement<FeatureState>(std::make_tuple(feat.x(), feat.y(), feat.z()))));

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
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsic))),
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
      for (const auto& id : feat_ids)
      {
        feat_initializer_vector.emplace_back(
            std::make_pair(id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));
      }

      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq, St))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsic))),
          feat_initializer_vector);

      MatrixEquality(state.T().asMatrix(), Matrix5::Identity());
      MatrixEquality(state.b(), Vector6::Zero());
      MatrixEquality(state.S().asMatrix(), S);
      MatrixEquality(state.K().asMatrix(), K);
      for (const auto& id : feat_ids)
      {
        MatrixEquality(state.f(id), feat);
      }
    }

    // Copy and assignment
    {
      std::vector<std::pair<SystemState::SystemStateKey, SystemStateElementUniquePtr>> feat_initializer_vector;
      for (const auto& id : feat_ids)
      {
        feat_initializer_vector.emplace_back(
            std::make_pair(id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));
      }

      SystemState state(
          opts.state_options_,
          std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
          std::make_pair(SystemStateElementName::S,
                         createSystemStateElement<CameraExtrinsicState>(std::make_tuple(Sq, St))),
          std::make_pair(SystemStateElementName::K,
                         createSystemStateElement<CameraIntrinsicState>(std::make_tuple(intrinsic))),
          feat_initializer_vector);

      SystemState state_copy(state);
      SystemStateEquality(state, state_copy, feat_ids);

      SystemState state_copy_2 = state;
      SystemStateEquality(state, state_copy_2, feat_ids);
    }
  }

  // Assertation tests
  {
    ASSERT_DEATH(
        {
          SystemState state(opts.state_options_,
                            std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(
                                                                          std::make_tuple(Quaternion::UnitRandom()))));
        },
        "");
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
      MSCEqFStateEquality(state, state_copy);

      MSCEqFState state_copy_2 = state;
      MSCEqFStateEquality(state, state_copy_2);
    }
  }
}

}  // namespace msceqf

#endif  // TEST_STATE_HPP