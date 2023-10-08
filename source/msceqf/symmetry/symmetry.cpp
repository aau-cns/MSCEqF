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

#include <unsupported/Eigen/MatrixFunctions>

#include "msceqf/symmetry/symmetry.hpp"

namespace msceqf
{
const Matrix5 Symmetry::D = []() {
  Matrix5 D = Matrix5::Zero();
  D(3, 4) = 1.0;
  return D;
}();

const SystemState Symmetry::phi(const MSCEqFState& X, const SystemState& xi)
{
  SE3 PS = xi.P();

  if (X.opts().num_persistent_features_ > 0)
  {
    PS.multiplyRight(xi.S());
  }

  SystemState result(xi);

  for (auto& [key, ptr] : result.state_)
  {
    assert(key.valueless_by_exception() == false);

    if (std::holds_alternative<SystemStateElementName>(key))
    {
      switch (std::get<SystemStateElementName>(key))
      {
        case SystemStateElementName::T:
          std::static_pointer_cast<ExtendedPoseState>(ptr)->T_.multiplyRight(X.D());
          break;
        case SystemStateElementName::b:
          std::static_pointer_cast<BiasState>(ptr)->b_ = X.B().invAdjoint() * (xi.b() - X.delta());
          break;
        case SystemStateElementName::S:
          std::static_pointer_cast<CameraExtrinsicState>(ptr)->S_.multiplyLeft(X.C().inv());
          std::static_pointer_cast<CameraExtrinsicState>(ptr)->S_.multiplyRight(X.E());
          break;
        case SystemStateElementName::K:
          std::static_pointer_cast<CameraIntrinsicState>(ptr)->K_.multiplyRight(X.L());
          break;
      }
    }
    else
    {
      std::static_pointer_cast<FeatureState>(ptr)->f_ =
          PS * (X.Q(std::get<uint>(key)).inv() * (PS.inv() * xi.f(std::get<uint>(key))));
    }
  }

  return result;
}

const SystemState::SystemStateAlgebraMap Symmetry::lift(const SystemState& xi, const Imu& u)
{
  SystemState::SystemStateAlgebraMap lambda(xi.state_.size());

  Vector9 lambda_T = Vector9::Zero();
  Vector6 lambda_S = Vector6::Zero();

  lambda_T.block<3, 1>(0, 0) = u.ang_ - xi.b().block<3, 1>(0, 0);
  lambda_T.block<3, 1>(3, 0) = u.acc_ - xi.b().block<3, 1>(3, 0) + xi.T().R().transpose() * xi.ge3();
  lambda_T.block<3, 1>(6, 0) = xi.T().R().transpose() * xi.T().v();

  // Precompute Lambda_S if we are estimating camera extrinsics or if we have persistent features
  // Note that the S() method provide either the initial value or the actual estimate of the extrinsics
  if (xi.opts_.enable_camera_extrinsics_calibration_ || xi.opts_.num_persistent_features_ > 0)
  {
    lambda_S = xi.S().invAdjoint() * (Vector6() << lambda_T.block<3, 1>(0, 0), lambda_T.block<3, 1>(6, 0)).finished();
  }

  SE3 PS_inv = xi.S().inv();

  if (xi.opts_.num_persistent_features_ > 0)
  {
    PS_inv.multiplyRight(xi.P().inv());
  }

  for (auto& [key, ptr] : xi.state_)
  {
    assert(key.valueless_by_exception() == false);

    if (std::holds_alternative<SystemStateElementName>(key))
    {
      switch (std::get<SystemStateElementName>(key))
      {
        case SystemStateElementName::T:
          lambda[key] = lambda_T;
          break;
        case SystemStateElementName::b:
          lambda[key] = SE3::adjoint(xi.b()) * lambda_T.block<6, 1>(0, 0);
          break;
        case SystemStateElementName::S:
          lambda[key] = lambda_S;
          break;
        case SystemStateElementName::K:
          lambda[key] = Vector4::Zero();
          break;
      }
    }
    else
    {
      // Feature expressed in camera frame and its squared depth
      Vector3 Cf = PS_inv * xi.f(std::get<uint>(key));
      fp Cf_depth2 = Cf.squaredNorm();

      // origin expressed in camera frame
      Vector3 Cnu = PS_inv * Vector3(Vector3::Zero());

      Vector4 lambda_f = Vector4::Zero();
      lambda_f.block<3, 1>(0, 0) = lambda_S.block<3, 1>(0, 0) + Cf.cross(lambda_S.block<3, 1>(3, 0) - Cnu) / Cf_depth2;
      lambda_f(3) = Cf.dot(lambda_S.block<3, 1>(3, 0) - Cnu) / Cf_depth2;
      lambda[key] = lambda_f;
    }
  }

  return lambda;
}

const MatrixX Symmetry::curvatureCorrection(const MSCEqFState& X, const VectorX& inn)
{
  MatrixX Gamma = MatrixX::Zero(inn.rows(), inn.rows());

  Gamma.block(X.index(MSCEqFStateElementName::Dd), X.index(MSCEqFStateElementName::Dd), 9, 9) =
      SE23::adjoint(inn.segment(X.index(MSCEqFStateElementName::Dd), 9));

  Gamma.block(X.index(MSCEqFStateElementName::Dd) + 9, X.index(MSCEqFStateElementName::Dd), 6, 6) =
      SE3::adjoint(inn.segment(X.index(MSCEqFStateElementName::Dd) + 9, 6));

  Gamma.block(X.index(MSCEqFStateElementName::Dd) + 9, X.index(MSCEqFStateElementName::Dd) + 9, 6, 6) =
      SE3::adjoint(inn.segment(X.index(MSCEqFStateElementName::Dd), 6));

  if (X.opts().enable_camera_extrinsics_calibration_)
  {
    Gamma.block(X.index(MSCEqFStateElementName::E), X.index(MSCEqFStateElementName::E), 6, 6) =
        SE3::adjoint(inn.segment(X.index(MSCEqFStateElementName::E), X.dof(MSCEqFStateElementName::E)));
  }

  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    Gamma.block(X.index(MSCEqFStateElementName::L), X.index(MSCEqFStateElementName::L), 4, 4) =
        In::adjoint(inn.segment(X.index(MSCEqFStateElementName::L), X.dof(MSCEqFStateElementName::L)));
  }

  for (auto& [timestamp, clone] : X.clones_)
  {
    Gamma.block(clone->getIndex(), clone->getIndex(), 6, 6) =
        SE3::adjoint(inn.segment(clone->getIndex(), clone->getDof()));
  }

  Gamma *= 0.5;

  return Gamma.exp();
}

}  // namespace msceqf