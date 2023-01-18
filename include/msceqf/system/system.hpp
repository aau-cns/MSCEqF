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

#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <variant>

#include "msceqf/options/msceqf_options.hpp"
#include "msceqf/state/state.hpp"
#include "msceqf/system/system_elements.hpp"

namespace msceqf
{
/**
 * @brief The SystemState class represent the state of the system posed on the Homogenous space.
 *
 * @note The system state has not to be confused with the MSCEqF state. The former is the state of the system posed on
 * the homogenous space, while the latter is the state of the lifted system, in which the EqF is based on.
 */
class SystemState
{
 public:
  using SystemStateKey = std::variant<SystemStateElementName, uint>;  //!< Key to access the system state map
  using SystemStateMap = std::unordered_map<SystemStateKey, SystemStateElementSharedPtr>;  //!< System state map

  /**
   * @brief Deleted default constructor
   */
  SystemState() = delete;

  /**
   * @brief Construct system state given a multiple pairs of key-pointer of states element. This methods preallocate
   * memory for the state_ map and insert the given pointers.
   * Camera intrinsics and extrinsics are initialized from the given values in the options, passing pairs of key-pointer
   * of camera intrinsics and extrinsics will overwrite the intrinsics and extrinsics initialized form the given options.
   *
   * @tparam Args
   * @param opts State options
   * @param keys_args multiple pairs of key-ptr where each pointer is pointing to a state elements
   *
   * @note Examples of call:
   * @note 1) SystemState(opts, pair(key1, ptr1), pair(key2, ptr2), pair(key3, ptr3))
   * @note 2) SystemState(opts, pair(key1, ptr1), vector(pair(key2, ptr2), ..., pair(key10, ptr10)))
   * @note When the function is called with a vector of pairs, the vector of pairs is moved therefore after the call
   * it contains only unusable pointers (nullptr).
   */
  template <typename... Args>
  SystemState(const StateOptions& opts, Args&&... pairs_of_key_ptr) : state_(), g_(opts.gravity_), opts_(opts)
  {
    // Preallocate state memory based on given options
    preallocate();

    // Initialize camera extrinsics and intrinsics;
    if (opts.enable_camera_extrinsics_calibration_)
    {
      insertSystemStateElement(std::make_pair(
          SystemStateElementName::S,
          createSystemStateElement<CameraExtrinsicState>(std::make_tuple(opts.initial_camera_extrinsics_))));
    }
    if (opts.enable_camera_intrinsics_calibration_)
    {
      insertSystemStateElement(std::make_pair(
          SystemStateElementName::K,
          createSystemStateElement<CameraIntrinsicState>(std::make_tuple(opts.initial_camera_intrinsics_))));
    }

    // Insert system state element into state_ map
    (insertSystemStateElement(std::forward<decltype(pairs_of_key_ptr)>(pairs_of_key_ptr)), ...);
  }

  /// Rule of Five
  SystemState(const SystemState& other);
  SystemState(SystemState&& other) noexcept;
  SystemState& operator=(const SystemState& other);
  SystemState& operator=(SystemState&& other) noexcept;
  ~SystemState();

  /**
   * @brief return a constant reference to the extended pose element (R,v,p) of the system state as a SE23-torsor
   *
   * @return const SE23&
   */
  [[nodiscard]] const SE23& T() const;

  /**
   * @brief return a copy of the pose element (R,p) of the system state as a SE3-torsor
   *
   * @return const SE3&
   */
  [[nodiscard]] const SE3 P() const;

  /**
   * @brief return a copy of the pose element (R,v) of the system state as a SE3-torsor
   *
   * @return const SE3&
   */
  [[nodiscard]] const SE3 V() const;

  /**
   * @brief return a constant reference to the bias element of the system state as a vector
   *
   * @return const Vector6&
   */
  [[nodiscard]] const Vector6& b() const;

  /**
   * @brief return a constant reference to the camera extrinsics element of the system state as a SE3-torsor.
   * If the camera extrinsics are not estimated online then the fixed calibration value provided in the options is
   * returned
   *
   * @return const SE3&
   */
  [[nodiscard]] const SE3& S() const;

  /**
   * @brief return a constant reference to the camera intrinsics element of the system state as a In-torsor
   * If the camera intrinsics are not are not estimated online then the fixed calibration value provided in the options
   * is returned
   *
   * @return const In&
   */
  [[nodiscard]] const In& K() const;

  /**
   * @brief return a constant reference to a persistent feature element of the system state as a vector, given the
   * feature id
   *
   * @param feat_id id of the persistent feature
   * @return const Vector3&
   */
  [[nodiscard]] const Vector3& f(const uint& feat_id) const;

  /**
   * @brief return a copy of g*e3 as a vector
   *
   * @return const Vector3
   */
  [[nodiscard]] const Vector3 ge3() const;

  /**
   * @brief Get a string describing the given SystemStateKey
   *
   * @param key
   * @return std::string
   */
  static std::string toString(const SystemStateKey& key);

 private:
  /**
   * @brief Preallocate space on the system state map based on given options
   *
   */
  void preallocate();

  /**
   * @brief Insert a single element into the state map given a pair of key-ptr. Each pointer points to a state
   * element ot be inserted into the state_ map
   *
   * @param key_ptr pair of key-ptr
   */
  void insertSystemStateElement(std::pair<SystemStateKey, SystemStateElementUniquePtr>&& key_ptr);

  /**
   * @brief Insert a multi element into the state map given a vector of pairs of key-ptr. Each pointer points to a
   * state element ot be inserted into the state_ map
   *
   * @param keys_ptrs vector of pairs of key-ptr.
   */
  void insertSystemStateElement(std::vector<std::pair<SystemStateKey, SystemStateElementUniquePtr>>& keys_ptrs);

  friend class Symmetry;  //!< Symmetry can access private members of SystemState

  SystemStateMap state_;  //!< MSCEqF State elements mapped by their names
  fp g_;                  //!< The magnitude of the gravity vector. in m/s^2 (The direction is e3 by default)

 public:
  StateOptions opts_;  //!< State Options
};

}  // namespace msceqf

#endif  // SYSTEM_HPP

// [COMMENT] I can use unorderd map as soon as i check for out of order images when getting a image...