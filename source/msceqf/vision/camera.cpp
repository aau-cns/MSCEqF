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

namespace msceqf
{
PinholeCamera::PinholeCamera(const VectorX& distortion_coefficients,
                             const Vector4 instrinsics,
                             const uint& width,
                             const uint& height)
    : distortion_coefficients_(distortion_coefficients), intrinsics_(instrinsics), width_(width), height_(height)
{
}

void PinholeCamera::setIntrinsics(const Vector4& intrinsics) { intrinsics_ = intrinsics; }

RadtanCamera::RadtanCamera(const CameraOptions& opts, const Vector4& intrinsics)
    : PinholeCamera(opts.distortion_coefficients_, intrinsics, opts.resolution_(0), opts.resolution_(1))
{
}

Vector2 RadtanCamera::undistort(const Vector2& distorted_uv)
{
  cv::Vec<fp, 2> uvn_cv;
  cv::eigen2cv(distorted_uv, uvn_cv);

  return undistort(uvn_cv);
}

Vector2 RadtanCamera::undistort(const cv::Vec<fp, 2>& distorted_uv)
{
  cv::Vec<fp, 2> uvn_cv;
  cv::Vec<fp, 4> dist_cv;
  cv::Matx<fp, 3, 3> K_cv;

  cv::eigen2cv(distortion_coefficients_, dist_cv);

  K_cv(0, 0) = intrinsics_(0);
  K_cv(1, 1) = intrinsics_(1);
  K_cv(0, 2) = intrinsics_(2);
  K_cv(1, 2) = intrinsics_(3);

  cv::undistortPoints(distorted_uv, uvn_cv, K_cv, dist_cv);

  Vector2 normalized_uv;
  cv::cv2eigen(uvn_cv, normalized_uv);

  return normalized_uv;
}

}  // namespace msceqf