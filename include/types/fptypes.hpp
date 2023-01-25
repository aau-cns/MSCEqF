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

#ifndef FPTYPES_HPP
#define FPTYPES_HPP

#include "groups/In.hpp"
#include "groups/SDB.hpp"
#include "groups/SOT3.hpp"

namespace msceqf
{

#ifdef SINGLE_PRECISION
using SO3 = group::SO3f;
using SE3 = group::SE3f;
using SE23 = group::SE23f;
using SOT3 = group::SOT3f;
using In = group::Inf;
using SDB = group::SDBf;
using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;
using Vector6 = Eigen::Matrix<float, 6, 1>;
using Vector9 = Eigen::Matrix<float, 9, 1>;
using Vector15 = Eigen::Matrix<float, 15, 1>;
using Vector21 = Eigen::Matrix<float, 21, 1>;
using Vector25 = Eigen::Matrix<float, 25, 1>;
using VectorX = Eigen::VectorXf;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Matrix5 = Eigen::Matrix<float, 5, 5>;
using Matrix6 = Eigen::Matrix<float, 6, 6>;
using Matrix9 = Eigen::Matrix<float, 9, 9>;
using Matrix12 = Eigen::Matrix<float, 12, 12>;
using Matrix15 = Eigen::Matrix<float, 15, 15>;
using Matrix21 = Eigen::Matrix<float, 21, 21>;
using Matrix25 = Eigen::Matrix<float, 25, 25>;
using MatrixX = Eigen::MatrixXf;
using Quaternion = Eigen::Quaternionf;
using fp = float;
#else
using SO3 = group::SO3d;
using SE3 = group::SE3d;
using SE23 = group::SE23d;
using SOT3 = group::SOT3d;
using In = group::Ind;
using SDB = group::SDBd;
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector9 = Eigen::Matrix<double, 9, 1>;
using Vector15 = Eigen::Matrix<double, 15, 1>;
using Vector21 = Eigen::Matrix<double, 21, 1>;
using Vector25 = Eigen::Matrix<double, 25, 1>;
using VectorX = Eigen::VectorXd;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Matrix5 = Eigen::Matrix<double, 5, 5>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Matrix9 = Eigen::Matrix<double, 9, 9>;
using Matrix12 = Eigen::Matrix<double, 12, 12>;
using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Matrix21 = Eigen::Matrix<double, 21, 21>;
using Matrix25 = Eigen::Matrix<double, 25, 25>;
using MatrixX = Eigen::MatrixXd;
using Quaternion = Eigen::Quaterniond;
using fp = double;
#endif

}  // namespace msceqf

#endif  // FPTYPES_HPP
