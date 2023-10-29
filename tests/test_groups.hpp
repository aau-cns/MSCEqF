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

#ifndef TEST_GROUPS_HPP
#define TEST_GROUPS_HPP

#include <groups/In.hpp>
#include <groups/SDB.hpp>
#include <groups/SOT3.hpp>
#include <groups/TG.hpp>

namespace msceqf
{
using namespace group;

typedef testing::Types<SO3f, SO3d, SOT3f, SOT3d, SE3d, SE3f, SE23f, SE23d, Inf, Ind> MSCEqFBaseGroups;
typedef testing::Types<SO3f, SO3d, SOT3f, SOT3d, SE3d, SE3f, SE23f, SE23d> MSCEqFBaseGroupsWithJacobians;

/**
 * @brief MSCEqF base groups specific tests
 */
template <typename T>
class MSCEqFBaseGroupsTest : public testing::Test
{
};
TYPED_TEST_SUITE(MSCEqFBaseGroupsTest, MSCEqFBaseGroups);

TYPED_TEST(MSCEqFBaseGroupsTest, TestExpLog)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    typename TypeParam::VectorType x = TypeParam::VectorType::Zero();
    typename TypeParam::VectorType y = TypeParam::log(TypeParam::exp(x));
    MatrixEquality(x, y);

    x = 1e-12 * TypeParam::VectorType::Random();
    auto X = TypeParam::exp(x);
    y = TypeParam::log(X);
    MatrixEquality(x, y);

    x = TypeParam::VectorType::Random();
    y = TypeParam::log(TypeParam::exp(x));
    MatrixEquality(x, y);
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestWedgeVee)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    typename TypeParam::VectorType u = TypeParam::VectorType::Random();
    typename TypeParam::MatrixType U = TypeParam::wedge(u);
    typename TypeParam::VectorType v = TypeParam::vee(U);
    MatrixEquality(u, v);
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestAssociativity)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    auto X1 = TypeParam::exp(TypeParam::VectorType::Random());
    auto X2 = TypeParam::exp(TypeParam::VectorType::Random());
    auto X3 = TypeParam::exp(TypeParam::VectorType::Random());

    auto Z1 = (X1 * X2) * X3;
    auto Z2 = X1 * (X2 * X3);

    MatrixEquality(Z1.asMatrix(), Z2.asMatrix());
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestIdentity)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    auto X = TypeParam::exp(TypeParam::VectorType::Random());
    auto I = TypeParam();
    typename TypeParam::MatrixType Imat = TypeParam::MatrixType::Identity();

    MatrixEquality(I.asMatrix(), Imat);

    auto X1 = X * I;
    auto X2 = I * X;

    MatrixEquality(X.asMatrix(), X1.asMatrix());
    MatrixEquality(X.asMatrix(), X2.asMatrix());
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestInverse)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    auto X = TypeParam::exp(TypeParam::VectorType::Random());
    auto X_inv = X.inv();
    auto I = TypeParam();

    auto I1 = X * X_inv;
    auto I2 = X_inv * X;

    MatrixEquality(I.asMatrix(), I1.asMatrix());
    MatrixEquality(I.asMatrix(), I2.asMatrix());
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestGroupProduct)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    auto X = TypeParam::exp(TypeParam::VectorType::Random());
    auto Y = TypeParam::exp(TypeParam::VectorType::Random());

    typename TypeParam::MatrixType Z1 = (X * Y).asMatrix();
    typename TypeParam::MatrixType Z2 = X.asMatrix() * Y.asMatrix();

    MatrixEquality(Z1, Z2);
  }
  {
    auto X = TypeParam::exp(TypeParam::VectorType::Random());
    auto Y = TypeParam::exp(TypeParam::VectorType::Random());

    auto Z = X * Y;
    auto W = Y * X;

    auto X1 = X;
    auto X2 = X;

    X1.multiplyRight(Y);
    X2.multiplyLeft(Y);

    MatrixEquality(Z.asMatrix(), X1.asMatrix());
    MatrixEquality(W.asMatrix(), X2.asMatrix());
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestGroupAlgebraProduct)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    auto X = TypeParam::exp(TypeParam::VectorType::Random());
    auto U = TypeParam::wedge(TypeParam::VectorType::Random());

    typename TypeParam::MatrixType V1 = X * U;
    typename TypeParam::MatrixType V2 = X.asMatrix() * U;

    MatrixEquality(V1, V2);
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestGroupAdjoint)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    auto X = TypeParam::exp(TypeParam::VectorType::Random());
    typename TypeParam::VectorType u = TypeParam::VectorType::Random();

    auto Ad = X.Adjoint();
    auto invAd = X.invAdjoint();

    auto v1 = Ad * u;
    auto v2 = TypeParam::vee(X.asMatrix() * TypeParam::wedge(u) * X.inv().asMatrix());

    MatrixEquality(v1, v2);
    MatrixEquality(Ad.inverse(), invAd);
  }
}

TYPED_TEST(MSCEqFBaseGroupsTest, TestAlgebraAdjoint)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    typename TypeParam::VectorType u = TypeParam::VectorType::Random();
    typename TypeParam::VectorType v = TypeParam::VectorType::Random();

    typename TypeParam::VectorType ad1 = TypeParam::adjoint(u) * v;
    typename TypeParam::VectorType ad2 =
        TypeParam::vee(TypeParam::wedge(u) * TypeParam::wedge(v) - TypeParam::wedge(v) * TypeParam::wedge(u));

    MatrixEquality(ad1, ad2);
  }
}

/**
 * @brief MSCEqF base groups with Jacobians specific tests
 */
template <typename T>
class MSCEqFBaseGroupsWithJacobiansTest : public testing::Test
{
};
TYPED_TEST_SUITE(MSCEqFBaseGroupsWithJacobiansTest, MSCEqFBaseGroupsWithJacobians);

TYPED_TEST(MSCEqFBaseGroupsWithJacobiansTest, leftJacobian)
{
  for (int i = 0; i < N_TESTS; ++i)
  {
    typename TypeParam::VectorType u = TypeParam::VectorType::Random();

    auto X = TypeParam::exp(u);

    typename TypeParam::TMatrixType A = X.Adjoint();
    typename TypeParam::TMatrixType B =
        TypeParam::TMatrixType::Identity() + TypeParam::adjoint(u) * TypeParam::leftJacobian(u);

    MatrixEquality(A, B);
  }
}

}  // namespace msceqf

#endif  // TEST_GROUPS_HPP