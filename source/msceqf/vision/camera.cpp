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

#include "msceqf/vision/camera.hpp"

#include <opencv2/core/eigen.hpp>

namespace msceqf::vision
{
PinholeCamera::PinholeCamera(const VectorX& distortion_coefficients,
                             const Vector4 instrinsics,
                             const uint& width,
                             const uint& height)
    : distortion_coefficients_(distortion_coefficients), intrinsics_(instrinsics), width_(width), height_(height)
{
}

void PinholeCamera::setIntrinsics(const Vector4& intrinsics) { intrinsics_ = intrinsics; }

void PinholeCamera::normalize(std::vector<Eigen::Vector2f>& uv)
{
  for (auto& coords : uv)
  {
    coords(0) = (coords(0) - intrinsics_(2)) / intrinsics_(0);
    coords(1) = (coords(1) - intrinsics_(3)) / intrinsics_(1);
  }
}

void PinholeCamera::normalize(std::vector<cv::Point2f>& uv)
{
  for (auto& coords : uv)
  {
    coords.x = (coords.x - intrinsics_(2)) / intrinsics_(0);
    coords.y = (coords.y - intrinsics_(3)) / intrinsics_(1);
  }
}

void PinholeCamera::normalize(Eigen::Vector2f& uv)
{
  uv(0) = (uv(0) - intrinsics_(2)) / intrinsics_(0);
  uv(1) = (uv(1) - intrinsics_(3)) / intrinsics_(1);
}

void PinholeCamera::normalize(cv::Point2f& uv)
{
  uv.x = (uv.x - intrinsics_(2)) / intrinsics_(0);
  uv.y = (uv.y - intrinsics_(3)) / intrinsics_(1);
}

void PinholeCamera::denormalize(std::vector<Eigen::Vector2f>& uv)
{
  for (auto& coords : uv)
  {
    coords(0) = coords(0) * intrinsics_(0) + intrinsics_(2);
    coords(1) = coords(1) * intrinsics_(1) + intrinsics_(3);
  }
}

void PinholeCamera::denormalize(std::vector<cv::Point2f>& uv)
{
  for (auto& coords : uv)
  {
    coords.x = coords.x * intrinsics_(0) + intrinsics_(2);
    coords.y = coords.y * intrinsics_(1) + intrinsics_(3);
  }
}

void PinholeCamera::denormalize(Eigen::Vector2f& uv)
{
  uv(0) = uv(0) * intrinsics_(0) + intrinsics_(2);
  uv(1) = uv(1) * intrinsics_(1) + intrinsics_(3);
}

void PinholeCamera::denormalize(cv::Point2f& uv)
{
  uv.x = uv.x * intrinsics_(0) + intrinsics_(2);
  uv.y = uv.y * intrinsics_(1) + intrinsics_(3);
}

RadtanCamera::RadtanCamera(const CameraOptions& opts, const Vector4& intrinsics)
    : PinholeCamera(opts.distortion_coefficients_, intrinsics, opts.resolution_(0), opts.resolution_(1))
{
}

void RadtanCamera::undistort(std::vector<Eigen::Vector2f>& uv, const bool& normalize)
{
  std::vector<cv::Point2f> uv_cv;
  uv_cv.reserve(uv.size());

  for (const auto& coords : uv)
  {
    uv_cv.emplace_back(coords(0), coords(1));
  }

  undistort(uv_cv, normalize);

  for (size_t i = 0; i < uv_cv.size(); ++i)
  {
    uv[i](0) = uv_cv[i].x;
    uv[i](1) = uv_cv[i].y;
  }
}

void RadtanCamera::undistort(std::vector<cv::Point2f>& uv_cv, const bool& normalize)
{
  cv::Vec<fp, 4> dist_cv;
  cv::Matx<fp, 3, 3> K_cv;

  cv::eigen2cv(distortion_coefficients_, dist_cv);

  K_cv(0, 0) = intrinsics_(0);
  K_cv(1, 1) = intrinsics_(1);
  K_cv(0, 2) = intrinsics_(2);
  K_cv(1, 2) = intrinsics_(3);
  K_cv(2, 2) = 1.0f;

  if (normalize)
  {
    cv::undistortPoints(uv_cv, uv_cv, K_cv, dist_cv);
  }
  else
  {
    cv::undistortPoints(uv_cv, uv_cv, K_cv, dist_cv, cv::noArray(), K_cv);
  }
}

}  // namespace msceqf