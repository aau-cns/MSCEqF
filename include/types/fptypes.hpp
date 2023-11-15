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

#include <groups/In.hpp>
#include <groups/SDB.hpp>
#include <groups/TG.hpp>
#include <groups/SOT3.hpp>

namespace msceqf
{
#ifdef SINGLE_PRECISION

using fp = float;

#else

using fp = double;

#endif

using SO3 = group::SO3<fp>;

using SE3 = group::SEn3<fp, 1>;

using SE23 = group::SEn3<fp, 2>;

using SOT3 = group::SOT3<fp>;

using In = group::In<fp>;

using SDB = group::SemiDirectBias<fp>;

using TG = group::Tangent<fp>;

using Vector2 = Eigen::Matrix<fp, 2, 1>;

using Vector3 = Eigen::Matrix<fp, 3, 1>;

using Vector4 = Eigen::Matrix<fp, 4, 1>;

using Vector6 = Eigen::Matrix<fp, 6, 1>;

using Vector9 = Eigen::Matrix<fp, 9, 1>;

using Vector15 = Eigen::Matrix<fp, 15, 1>;

using Vector18 = Eigen::Matrix<fp, 18, 1>;

using Vector21 = Eigen::Matrix<fp, 21, 1>;

using Vector24 = Eigen::Matrix<fp, 24, 1>;

using Vector25 = Eigen::Matrix<fp, 25, 1>;

using Vector29 = Eigen::Matrix<fp, 29, 1>;

using VectorX = Eigen::Matrix<fp, Eigen::Dynamic, 1>;

using Matrix2 = Eigen::Matrix<fp, 2, 2>;

using Matrix3 = Eigen::Matrix<fp, 3, 3>;

using Matrix4 = Eigen::Matrix<fp, 4, 4>;

using Matrix5 = Eigen::Matrix<fp, 5, 5>;

using Matrix6 = Eigen::Matrix<fp, 6, 6>;

using Matrix7 = Eigen::Matrix<fp, 7, 7>;

using Matrix9 = Eigen::Matrix<fp, 9, 9>;

using Matrix12 = Eigen::Matrix<fp, 12, 12>;

using Matrix15 = Eigen::Matrix<fp, 15, 15>;

using Matrix18 = Eigen::Matrix<fp, 18, 18>;

using Matrix21 = Eigen::Matrix<fp, 21, 21>;

using Matrix24 = Eigen::Matrix<fp, 24, 24>;

using Matrix25 = Eigen::Matrix<fp, 25, 25>;

using Matrix29 = Eigen::Matrix<fp, 24, 24>;

using MatrixX = Eigen::MatrixXd;

using Quaternion = Eigen::Quaternion<fp>;

template <int R, int C, int T = Eigen::ColMajor>
using Matrix = Eigen::Matrix<fp, R, C, T>;

template <typename T>
using Ref = Eigen::Ref<T>;

template <typename T>
using Map = Eigen::Map<T>;

}  // namespace msceqf

#endif  // FPTYPES_HPP
