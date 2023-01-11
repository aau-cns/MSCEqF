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

#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <variant>

#include "msceqf/msceqf_options.hpp"
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
   * @brief Construct system state given a multiple pairs of key-pointer of states element. This methods preallocate
   * memory for the state_ map and insert the given pointers.
   *
   * @tparam Args
   * @param opts State options
   * @param keys_args multiple pairs of key-ptr where each pointer is pointing to a state elements
   *
   * @note Examples of call:
   * @note 1) SystemState(opts, pair(key1, ptr1), pair(key2, ptr2), pair(key3, ptr3))
   * @note 2) SystemState(opts, pair(key1, ptr1), vector(pair(key2, ptr2), ..., pair(key10, ptr10)))
   */
  template <typename... Args>
  SystemState(const StateOptions& opts, Args&&... pairs_of_key_ptr) : state_()
  {
    // Preallocate state memory based on given options
    size_t num_elements = 1 + opts.num_persistent_features_;
    if (opts.enable_camera_extrinsic_calibration_)
    {
      ++num_elements;
    }
    if (opts.enable_camera_intrinsic_calibration_)
    {
      ++num_elements;
    }
    state_.reserve(num_elements);

    // Insert system state element into state_ map
    (insertSystemStateElement(std::forward<decltype(pairs_of_key_ptr)>(pairs_of_key_ptr)), ...);
  }

  SystemStateMap state_;  //!< MSCEqF State elements mapped by their names

 private:
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
};

}  // namespace msceqf

#endif  // SYSTEM_HPP

// [COMMENT] I can use unorderd map as soon as i check for out of order images when getting a image...