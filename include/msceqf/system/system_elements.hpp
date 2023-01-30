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

#ifndef SYSTEM_ELEMENTS_HPP
#define SYSTEM_ELEMENTS_HPP

#include <memory>

#include "types/fptypes.hpp"

namespace msceqf
{

/**
 * @brief This enum class define the names of the system state elements.
 * This is used to create a map of state element mapped by the name,
 * and retreive specific element of the state through a map.
 *
 * @note Note that the persisten feature are excluded since their are refeered to through their id
 */
enum class SystemStateElementName
{
  T,  //!< Name of the extended pose (R, v, p) element of the system state
  b,  //!< Name of the IMU bias (bw, ba) element of the system state
  S,  //!< Name of the camera extrinsics (SR, St) element of the system state
  K,  //!< Name of the camera intrinsics (K) element of the system state
};

/**
 * @brief This struct represent the base struct for a general element of the system state.
 *
 */
class SystemStateElement
{
 public:
  virtual ~SystemStateElement() = default;

  /**
   * @brief Clone
   *
   * @return std::unique_ptr<SystemStateElement>
   */
  virtual std::unique_ptr<SystemStateElement> clone() const = 0;

 protected:
  /// @brief Rule of Five
  SystemStateElement() = default;
  SystemStateElement(const SystemStateElement&) = default;
  SystemStateElement(SystemStateElement&&) = default;
  SystemStateElement& operator=(const SystemStateElement&) = default;
  SystemStateElement& operator=(SystemStateElement&&) = default;
};

/**
 * @brief This struct represent the extended pose state of the system
 *
 */
struct ExtendedPoseState final : public SystemStateElement
{
  ExtendedPoseState() : T_(){};
  ExtendedPoseState(const SE23& T) : T_(T){};

  /**
   * @brief Clone the extended pose state element of the system
   *
   * @return std::unique_ptr<SystemStateElement>
   */
  std::unique_ptr<SystemStateElement> clone() const override { return std::make_unique<ExtendedPoseState>(*this); }

  SE23 T_;  //!< The extended pose of the system (R, v, p)
};

/**
 * @brief This struct represent the IMU bias state of the system
 *
 */
struct BiasState final : public SystemStateElement
{
  BiasState() : b_(Vector6::Zero()){};
  BiasState(const Vector6& b) : b_(b){};

  /**
   * @brief Clone the bias state element of the system
   *
   * @return std::unique_ptr<SystemStateElement>
   */
  std::unique_ptr<SystemStateElement> clone() const override { return std::make_unique<BiasState>(*this); }

  Vector6 b_;  //!< The Inertial Measurement Unit (IMU) biases (bw, ba)
};

/**
 * @brief This struct represent the camera extrinsics state of the system
 *
 */
struct CameraExtrinsicState final : public SystemStateElement
{
  CameraExtrinsicState() : S_(){};
  CameraExtrinsicState(const Quaternion& q, const Vector3& t) : S_(q, {t}){};
  CameraExtrinsicState(const Matrix3& R, const Vector3& t) : S_(R, {t}){};
  CameraExtrinsicState(const Matrix4& S) : S_(S){};
  CameraExtrinsicState(const SE3& S) : S_(S){};

  /**
   * @brief Clone the camera extrinsics state element of the system
   *
   * @return std::unique_ptr<SystemStateElement>
   */
  std::unique_ptr<SystemStateElement> clone() const override { return std::make_unique<CameraExtrinsicState>(*this); }

  SE3 S_;  //!< The camera extrinsics calibration (SR, St)
};

/**
 * @brief This struct represent the camera intrinsics state of the system
 *
 */
struct CameraIntrinsicState final : public SystemStateElement
{
  CameraIntrinsicState() : K_(){};
  CameraIntrinsicState(const fp& fx, const fp& fy, const fp& cx, const fp& cy) : K_(fx, fy, cx, cy){};
  CameraIntrinsicState(const Vector4& intr) : K_(intr){};
  CameraIntrinsicState(const Matrix3& K) : K_(K){};
  CameraIntrinsicState(const In& K) : K_(K){};

  /**
   * @brief Clone the camera instirnsic state element of the system
   *
   * @return std::unique_ptr<SystemStateElement>
   */
  std::unique_ptr<SystemStateElement> clone() const override { return std::make_unique<CameraIntrinsicState>(*this); }

  In K_;  //!< The camera intrinsics calibration (K)
};

/**
 * @brief This struct represent a single persistent feature state of the system
 *
 */
struct FeatureState final : public SystemStateElement
{
 public:
  FeatureState() : f_(Vector3::Zero()){};
  FeatureState(const Vector3& f) : f_(f){};
  FeatureState(const fp& x, const fp& y, const fp& z) : f_({x, y, z}){};

  /**
   * @brief Clone the persistent feature state element of the system
   *
   * @return std::unique_ptr<SystemStateElement>
   */
  std::unique_ptr<SystemStateElement> clone() const override { return std::make_unique<FeatureState>(*this); }

  Vector3 f_;  //!< The persistent feature (f)
};

using SystemStateElementSharedPtr = std::shared_ptr<SystemStateElement>;
using SystemStateElementUniquePtr = std::unique_ptr<SystemStateElement>;
using ExtendedPoseStateSharedPtr = std::shared_ptr<ExtendedPoseState>;
using ExtendedPoseUniquePtr = std::unique_ptr<ExtendedPoseState>;
using BiasStateStateSharedPtr = std::shared_ptr<BiasState>;
using BiasStateStateUniquePtr = std::unique_ptr<BiasState>;
using CameraExtrinsicStateStateSharedPtr = std::shared_ptr<CameraExtrinsicState>;
using CameraExtrinsicStateStateUniquePtr = std::unique_ptr<CameraExtrinsicState>;
using CameraIntrinsicStateStateSharedPtr = std::shared_ptr<CameraIntrinsicState>;
using CameraIntrinsicStateStateUniquePtr = std::unique_ptr<CameraIntrinsicState>;
using FeatureStateStateStateSharedPtr = std::shared_ptr<FeatureState>;
using FeatureStateStateStateUniquePtr = std::unique_ptr<FeatureState>;

/**
 * @brief Factory method for system state elements.
 * This factory method is general in the sense that it allows to construct an unique pointer of all the
 * system state elements, given the possible multiple constructors for each state element.
 *
 * @tparam Args variadic arguments
 * @param name name of the system state element (see SystemStateElementName enum)
 * @param args arguments for state creation, argument used in the constructor of system state elements
 * @return std::unique_ptr<SystemStateElement>
 *
 * @note The function return a nullptr if the provided arguments do not fit any constructor
 */
template <typename T, typename... Args>
[[nodiscard]] static SystemStateElementUniquePtr createSystemStateElement(const std::tuple<Args...>& args)
{
  return std::apply(
      [](const auto&... args)
      {
        if constexpr (std::is_base_of_v<SystemStateElement, T> && std::is_constructible_v<T, decltype(args)...>)
        {
          return std::make_unique<T>(args...);
        }
        else
        {
          return nullptr;
        }
      },
      args);
}

}  // namespace msceqf

#endif  // SYSTEM_ELEMENTS_HPP