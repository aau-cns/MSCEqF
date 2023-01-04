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

using testing::Types;
using namespace std;
using namespace Eigen;

constexpr double EPS = 1e-6;
constexpr int N_TESTS = 100;

template <typename Derived, typename OtherDerived>
void MatrixEquality(const MatrixBase<Derived>& A, const MatrixBase<OtherDerived>& B)
{
  // The check on the norm is to cover the cases in which both A, and B are close to zero
  EXPECT_TRUE(A.isApprox(B, EPS) || (A - B).norm() < EPS);
}

template <typename FPType>
void QuaternionEquality(const Quaternion<FPType>& a, const Quaternion<FPType>& b)
{
  EXPECT_TRUE(a.coeffs().isApprox(b.coeffs(), EPS) || a.coeffs().isApprox(-b.coeffs(), EPS));
}

void ScalarEquality(const double& a, const double& b) { EXPECT_TRUE(norm(a - b) < EPS); }

double randDouble() { return static_cast<double>(rand()) / static_cast<double>(RAND_MAX); }

#endif  // TEST_COMMON_HPP