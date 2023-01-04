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

#ifndef FPTYPE_HPP
#define FPTYPE_HPP

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
using MatrixX = Eigen::MatrixXf;
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
using MatrixX = Eigen::MatrixXd;
#endif
}  // namespace msceqf

#endif  // FPTYPE_HPP
