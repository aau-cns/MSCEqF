// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
// System Theory and Robotics Lab, Australian Centre for Robotic
// Vision, Australian national University, Australia.
//
// All rights reserved.
//
// This software is licensed under the terms of the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>,
// <pieter.vangoor@anu.edu.au>.

#ifndef TEST_COMMON_HPP
#define TEST_COMMON_HPP

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "msceqf/state/state.hpp"

namespace msceqf
{
const std::string parameters_path = "../../tests/config/parameters.yaml";

constexpr fp EPS = 1e-6;
constexpr int N_TESTS = 100;

template <typename Derived, typename OtherDerived>
void MatrixEquality(const Eigen::MatrixBase<Derived>& A, const Eigen::MatrixBase<OtherDerived>& B, fp tol = EPS)
{
  // The check on the norm is to cover the cases in which both A, and B are close to zero
  EXPECT_TRUE(A.isApprox(B, tol) || (A - B).norm() < tol);
}

template <typename FPType>
void QuaternionEquality(const Eigen::Quaternion<FPType>& a, const Eigen::Quaternion<FPType>& b, fp tol = EPS)
{
  EXPECT_TRUE(a.coeffs().isApprox(b.coeffs(), tol) || a.coeffs().isApprox(-b.coeffs(), tol));
}

void ScalarEquality(const fp& a, const fp& b, fp tol = EPS) { EXPECT_TRUE(std::norm(a - b) < tol); }

void SystemStateEquality(const msceqf::SystemState& xi1,
                         const msceqf::SystemState& xi2,
                         const std::vector<uint>& feat_ids = std::vector<uint>(),
                         fp tol = EPS)
{
  MatrixEquality(xi1.T().asMatrix(), xi2.T().asMatrix(), tol);
  MatrixEquality(xi1.b(), xi2.b(), tol);

  MatrixEquality(xi1.S().asMatrix(), xi2.S().asMatrix(), tol);

  assert(xi1.opts().enable_camera_intrinsics_calibration_ == xi2.opts().enable_camera_intrinsics_calibration_);
  if (xi1.opts().enable_camera_intrinsics_calibration_)
  {
    MatrixEquality(xi1.K().asMatrix(), xi2.K().asMatrix(), tol);
  }

  for (const auto& id : feat_ids)
  {
    MatrixEquality(xi1.f(id), xi2.f(id), tol);
  }
}

void MSCEqFStateEquality(const msceqf::MSCEqFState& X1,
                         const msceqf::MSCEqFState& X2,
                         const std::vector<uint>& feat_ids = std::vector<uint>(),
                         fp tol = EPS)
{
  MatrixEquality(X1.D().asMatrix(), X2.D().asMatrix(), tol);
  MatrixEquality(X1.delta(), X2.delta(), tol);
  MatrixEquality(X1.covBlock(msceqf::MSCEqFStateElementName::Dd), X2.covBlock(msceqf::MSCEqFStateElementName::Dd), tol);

  MatrixEquality(X1.E().asMatrix(), X2.E().asMatrix(), tol);
  MatrixEquality(X1.covBlock(msceqf::MSCEqFStateElementName::E), X2.covBlock(msceqf::MSCEqFStateElementName::E), tol);

  assert(X1.opts().enable_camera_intrinsics_calibration_ == X2.opts().enable_camera_intrinsics_calibration_);
  if (X1.opts().enable_camera_intrinsics_calibration_)
  {
    MatrixEquality(X1.L().asMatrix(), X2.L().asMatrix(), tol);
    MatrixEquality(X1.covBlock(msceqf::MSCEqFStateElementName::L), X2.covBlock(msceqf::MSCEqFStateElementName::L), tol);
  }

  for (const auto& id : feat_ids)
  {
    MatrixEquality(X1.Q(id).asMatrix(), X2.Q(id).asMatrix(), tol);
    MatrixEquality(X1.covBlock(id), X2.covBlock(id), tol);
  }
}

}  // namespace msceqf

#endif  // TEST_COMMON_HPP