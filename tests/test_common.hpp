// Copyright (C) 2023 Alessandro Fornasier, Pieter van Goor.
// Control of Networked Systems, University of Klagenfurt, Austria.
// System Theory and Robotics Lab, Australian Centre for Robotic
// Vision, Australian national University, Australia.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>,
// <pieter.vangoor@anu.edu.au>.

#ifndef TEST_COMMON_HPP
#define TEST_COMMON_HPP

#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

#include "msceqf/state/state.hpp"
#include "msceqf/system/system.hpp"
#include "utils/tools.hpp"

namespace msceqf
{

constexpr double EPS = 1e-6;
constexpr int N_TESTS = 100;

template <typename Derived, typename OtherDerived>
void MatrixEquality(const Eigen::MatrixBase<Derived>& A, const Eigen::MatrixBase<OtherDerived>& B)
{
  // The check on the norm is to cover the cases in which both A, and B are close to zero
  EXPECT_TRUE(A.isApprox(B, EPS) || (A - B).norm() < EPS);
}

template <typename FPType>
void QuaternionEquality(const Eigen::Quaternion<FPType>& a, const Eigen::Quaternion<FPType>& b)
{
  EXPECT_TRUE(a.coeffs().isApprox(b.coeffs(), EPS) || a.coeffs().isApprox(-b.coeffs(), EPS));
}

void ScalarEquality(const fp& a, const fp& b) { EXPECT_TRUE(std::norm(a - b) < EPS); }

void SystemStateEquality(const msceqf::SystemState& xi1,
                         const msceqf::SystemState& xi2,
                         const std::vector<uint>& feat_ids = std::vector<uint>())
{
  MatrixEquality(xi1.T().asMatrix(), xi2.T().asMatrix());
  MatrixEquality(xi1.b(), xi2.b());

  assert(xi1.opts_.enable_camera_extrinsic_calibration_ == xi2.opts_.enable_camera_extrinsic_calibration_);
  if (xi1.opts_.enable_camera_extrinsic_calibration_)
  {
    MatrixEquality(xi1.S().asMatrix(), xi2.S().asMatrix());
  }

  assert(xi1.opts_.enable_camera_intrinsic_calibration_ == xi2.opts_.enable_camera_intrinsic_calibration_);
  if (xi1.opts_.enable_camera_intrinsic_calibration_)
  {
    MatrixEquality(xi1.K().asMatrix(), xi2.K().asMatrix());
  }

  for (const auto& id : feat_ids)
  {
    MatrixEquality(xi1.f(id), xi2.f(id));
  }
}

void MSCEqFStateEquality(const msceqf::MSCEqFState& X1,
                         const msceqf::MSCEqFState& X2,
                         const std::vector<uint>& feat_ids = std::vector<uint>())
{
  MatrixEquality(X1.D().asMatrix(), X2.D().asMatrix());
  MatrixEquality(X1.delta(), X2.delta());
  MatrixEquality(X1.CovBlock(msceqf::MSCEqFStateElementName::Dd), X2.CovBlock(msceqf::MSCEqFStateElementName::Dd));

  assert(X1.opts_.enable_camera_extrinsic_calibration_ == X2.opts_.enable_camera_extrinsic_calibration_);
  if (X1.opts_.enable_camera_extrinsic_calibration_)
  {
    MatrixEquality(X1.E().asMatrix(), X2.E().asMatrix());
    MatrixEquality(X1.CovBlock(msceqf::MSCEqFStateElementName::E), X2.CovBlock(msceqf::MSCEqFStateElementName::E));
  }
  assert(X1.opts_.enable_camera_intrinsic_calibration_ == X2.opts_.enable_camera_intrinsic_calibration_);
  if (X1.opts_.enable_camera_intrinsic_calibration_)
  {
    MatrixEquality(X1.L().asMatrix(), X2.L().asMatrix());
    MatrixEquality(X1.CovBlock(msceqf::MSCEqFStateElementName::L), X2.CovBlock(msceqf::MSCEqFStateElementName::L));
  }

  for (const auto& id : feat_ids)
  {
    MatrixEquality(X1.Q(id).asMatrix(), X2.Q(id).asMatrix());
    MatrixEquality(X1.CovBlock(id), X2.CovBlock(id));
  }
}

}  // namespace msceqf

#endif  // TEST_COMMON_HPP