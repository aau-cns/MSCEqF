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

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <memory>
#include <opencv2/opencv.hpp>

#include "msceqf/options/msceqf_options.hpp"
#include "types/fptypes.hpp"

namespace msceqf
{
/**
 * @brief This class represnt the base class for any pinhole camera type
 *
 */
class PinholeCamera
{
 public:
  virtual ~PinholeCamera() = default;

  /**
   * @brief Undistort given distorted point in Eigen format (std::vector<Eigen::Vector2f>)
   *
   * @param uv
   * @param normalize flag to decide wether normalize coordinates or not
   */
  virtual void undistort(std::vector<Eigen::Vector2f>& uv, const bool& normalize = false) = 0;

  /**
   * @brief Undistort given distorted point in OpenCV format (std::vector<cv::Point2f>)
   *
   * @param uv_cv
   * @param normalize flag to decide wether normalize coordinates or not
   */
  virtual void undistort(std::vector<cv::Point2f>& uv_cv, const bool& normalize = false) = 0;

  /**
   * @brief Set the value of the intrinsic parameters
   *
   * @param intrinsics
   */
  void setIntrinsics(const Vector4& intrinsics);

 protected:
  PinholeCamera(const VectorX& distortion_coefficients,
                const Vector4 instrinsics,
                const uint& width,
                const uint& height);

  /// @brief Rule of Five
  PinholeCamera() = delete;
  PinholeCamera(const PinholeCamera&) = delete;
  PinholeCamera(PinholeCamera&&) = delete;
  PinholeCamera& operator=(const PinholeCamera&) = delete;
  PinholeCamera& operator=(PinholeCamera&&) = delete;

  VectorX distortion_coefficients_;  //!< Vector of distortion coefficients (k1, k2, p1, p2, ...)
  Vector4 intrinsics_;               //!< Vector of intrinsic paramater (fx, fy, cx, cy)
  uint width_;                       //!< Image width
  uint height_;                      //!< Image height
};

/**
 * @brief This class represent a pinhole camera with radtan distortion model
 *
 */
struct RadtanCamera final : public PinholeCamera
{
  RadtanCamera(const CameraOptions& opts, const Vector4& intrinsics);

  /**
   * @brief Undistort given distorted point in Eigen format (std::vector<Eigen::Vector2f>)
   *
   * @param uv
   * @param normalize flag to decide wether normalize coordinates or not
   */
  void undistort(std::vector<Eigen::Vector2f>& uv, const bool& normalize) override;

  /**
   * @brief Undistort given distorted point in OpenCV format (std::vector<cv::Point2f>)
   *
   * @param uv_cv
   * @param normalize flag to decide wether normalize coordinates or not
   */
  void undistort(std::vector<cv::Point2f>& uv_cv, const bool& normalize) override;
};

// [TODO] Add support for other distortion model

using PinholeCameraSharedPtr = std::shared_ptr<PinholeCamera>;
using PinholeCameraUniquePtr = std::unique_ptr<PinholeCamera>;
using RadtanCameraSharedPtr = std::shared_ptr<RadtanCamera>;
using RadtanCameraUniquePtr = std::unique_ptr<RadtanCamera>;

/**
 * @brief Factory method for cameras
 *
 * @tparam underlying type of pointer to be made
 * @param opts camera options
 * @param intrinsics camera intrinsics as a 4-vector (fx, fy, cx, cy)
 * @return std::unique_ptr<PinholeCamera>
 */
template <typename T>
[[nodiscard]] static PinholeCameraUniquePtr createCamera(const CameraOptions& opts, const Vector4& intrinsics)
{
  if constexpr (std::is_base_of_v<PinholeCamera, T>)
  {
    return std::make_unique<T>(opts, intrinsics);
  }
  else
  {
    return nullptr;
  }
}

}  // namespace msceqf

#endif  // CAMERA_HPP