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

#ifndef TEST_SYMMETRY_HPP
#define TEST_SYMMETRY_HPP

#include <iostream>

#include "msceqf/symmetry/symmetry.hpp"

namespace msceqf
{

TEST(SymmetryTest, phi_without_persistent_features)
{
  // Param arser
  std::string filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";
  OptionParser parser(filepath_base + "parameters.yaml");

  // Options
  MSCEqFOptions opts = parser.parseOptions();

  for (int i = 0; i < N_TESTS; ++i)
  {
    // Set specific options for this test independently by given parameters
    opts.state_options_.enable_camera_extrinsics_calibration_ = static_cast<bool>(utils::random<int>(0, 1));
    opts.state_options_.enable_camera_intrinsics_calibration_ = static_cast<bool>(utils::random<int>(0, 1));
    opts.state_options_.num_persistent_features_ = 0;

    // Camera Extrinsic
    Quaternion Sq = Quaternion::UnitRandom();
    Vector3 St = Vector3::Random();
    opts.state_options_.initial_camera_extrinsics_ = SE3(Sq, {St});

    // Camera Intrinsic
    Vector4 intrinsics = Vector4::Random().cwiseAbs();
    opts.state_options_.initial_camera_intrinsics_ = In(intrinsics);

    // xi0 (implicit extrinsics and intrinsics in construction)
    SystemState xi0(
        opts.state_options_,
        std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
        std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())));

    // xi = xi0 = phi(I, xi0)
    MSCEqFState X(opts.state_options_);
    SystemState xi = Symmetry::phi(X, xi0);
    SystemStateEquality(xi, xi0);

    // phi(X2, phi(X1, xi)) = phi(X1*X2, xi)
    MSCEqFState X1(X.Random());
    MSCEqFState X2(X.Random());
    SystemState xi1 = Symmetry::phi(X2, Symmetry::phi(X1, xi));
    SystemState xi2 = Symmetry::phi(X1 * X2, xi);
    SystemStateEquality(xi1, xi2);
  }
}

TEST(SymmetryTest, lift)
{
  // Param arser
  std::string filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";
  OptionParser parser(filepath_base + "parameters.yaml");

  // Options
  MSCEqFOptions opts = parser.parseOptions();

  for (int i = 0; i < N_TESTS; ++i)
  {
    // Set specific options for this test independently by given parameters
    opts.state_options_.enable_camera_extrinsics_calibration_ = static_cast<bool>(utils::random<int>(0, 1));
    opts.state_options_.enable_camera_intrinsics_calibration_ = static_cast<bool>(utils::random<int>(0, 1));
    opts.state_options_.num_persistent_features_ = i == 0 ? 0 : utils::random<int>(0, 100);

    // Camera Extrinsic
    Quaternion Sq = Quaternion::UnitRandom();
    Vector3 St = Vector3::Random();
    opts.state_options_.initial_camera_extrinsics_ = SE3(Sq, {St});

    // Camera Intrinsic
    Vector4 intrinsics = Vector4::Random().cwiseAbs();
    opts.state_options_.initial_camera_intrinsics_ = In(intrinsics);

    // Feature
    Vector3 feat = Vector3::Random();

    // Features
    std::vector<uint> feat_ids;
    std::vector<std::pair<SystemState::SystemStateKey, SystemStateElementUniquePtr>> feat_initializer_vector;
    while (feat_ids.size() != opts.state_options_.num_persistent_features_)
    {
      uint id = utils::random<int>(0, 1000);
      if (std::find(feat_ids.begin(), feat_ids.end(), id) == feat_ids.end())
      {
        feat_ids.emplace_back(id);
        feat_initializer_vector.emplace_back(
            std::make_pair(id, createSystemStateElement<FeatureState>(std::make_tuple(feat))));
      }
    }

    // xi
    const SystemState xi(
        opts.state_options_,
        std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())),
        std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())),
        feat_initializer_vector);

    // u
    Imu u;
    u.ang_ = Vector3::Random();
    u.acc_ = Vector3::Random();

    // Continuous time dynamics
    Vector9 T_dot;
    T_dot.block<3, 1>(0, 0) = SO3::vee(xi.T().R() * SO3::wedge(u.ang_ - xi.b().block<3, 1>(0, 0)));
    T_dot.block<3, 1>(3, 0) = xi.T().R() * (u.acc_ - xi.b().block<3, 1>(3, 0)) + xi.ge3();
    T_dot.block<3, 1>(6, 0) = xi.T().v();
    Vector6 b_dot = Vector6::Zero();
    Vector6 S_dot = Vector6::Zero();
    Vector4 K_dot = Vector4::Zero();
    Vector3 f_dot = Vector3::Zero();

    // Lambda
    SystemState::SystemStateAlgebraMap lambda = Symmetry::lift(xi, u);

    // dphi* Lambda = xi_dot (T)
    MatrixEquality(SE23::vee(xi.T() * SE23::wedge(lambda.at(SystemStateElementName::T))), T_dot);

    // dphi* Lambda = xi_dot (b)
    MatrixEquality(SE3::adjoint(xi.b()) * lambda.at(SystemStateElementName::T).block<6, 1>(0, 0) -
                       lambda.at(SystemStateElementName::b),
                   b_dot);

    // dphi* Lambda = xi_dot (S)
    if (opts.state_options_.enable_camera_extrinsics_calibration_)
    {
      Vector6 lambda_P;
      lambda_P.block<3, 1>(0, 0) = lambda.at(SystemStateElementName::T).block<3, 1>(0, 0);
      lambda_P.block<3, 1>(3, 0) = lambda.at(SystemStateElementName::T).block<3, 1>(6, 0);
      MatrixEquality(SE3::vee(xi.S() * SE3::wedge(lambda.at(SystemStateElementName::S)) -
                              SE3::wedge(lambda_P) * xi.S().asMatrix()),
                     S_dot);
    }

    // dphi* Lambda = xi_dot (K)
    if (opts.state_options_.enable_camera_intrinsics_calibration_)
    {
      MatrixEquality(In::vee(xi.K() * In::wedge(lambda.at(SystemStateElementName::K))), K_dot);
    }

    // Precompute lambda_S for the case of no extrinsics calibration
    Vector6 lambda_S;
    if (opts.state_options_.enable_camera_extrinsics_calibration_)
    {
      lambda_S = lambda.at(SystemStateElementName::S);
    }
    else
    {
      lambda_S = xi.S().invAdjoint() * (Vector6() << lambda.at(SystemStateElementName::T).block<3, 1>(0, 0),
                                        lambda.at(SystemStateElementName::T).block<3, 1>(6, 0))
                                           .finished();
    }

    // Precompute feature independent values
    SE3 PS = xi.P() * xi.S();
    SE3 PS_inv = PS.inv();
    Vector3 alpha = lambda_S.block<3, 1>(0, 0);
    Vector3 beta = lambda_S.block<3, 1>(3, 0);

    // dphi* Lambda = xi_dot (f)
    for (const auto& id : feat_ids)
    {
      Vector3 Cf = PS_inv * xi.f(id);
      Vector3 gamma = lambda.at(id).block<3, 1>(0, 0);
      fp delta = lambda.at(id)(3);
      MatrixEquality(PS * Vector3((SO3::wedge(alpha - gamma) - delta * Matrix3::Identity()) * Cf + beta), f_dot);
    }
  }
}

}  // namespace msceqf

#endif  // TEST_SYMMETRY_HPP