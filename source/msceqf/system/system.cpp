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

#include "msceqf/system/system.hpp"

#include "utils/logger.hpp"

namespace msceqf
{
SystemState::SystemState(const StateOptions& opts) : opts_(opts), state_()
{
  preallocate();

  insertSystemStateElement(
      std::make_pair(SystemStateElementName::T, createSystemStateElement<ExtendedPoseState>(std::make_tuple())));
  insertSystemStateElement(
      std::make_pair(SystemStateElementName::b, createSystemStateElement<BiasState>(std::make_tuple())));

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
}

SystemState::SystemState(const SystemState& other) : opts_(other.opts_), state_()
{
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }
}

SystemState::SystemState(SystemState&& other) noexcept : opts_(std::move(other.opts_)), state_(std::move(other.state_))
{
}

SystemState& SystemState::operator=(const SystemState& other)
{
  state_.clear();
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }
  opts_ = other.opts_;
  return *this;
}

SystemState& SystemState::operator=(SystemState&& other) noexcept
{
  state_ = std::move(other.state_);
  opts_ = std::move(other.opts_);
  return *this;
}

SystemState ::~SystemState() { state_.clear(); }

void SystemState::preallocate()
{
  size_t num_elements = 1 + opts_.num_persistent_features_;
  if (opts_.enable_camera_extrinsics_calibration_)
  {
    ++num_elements;
  }
  if (opts_.enable_camera_intrinsics_calibration_)
  {
    ++num_elements;
  }
  state_.reserve(num_elements);
}

void SystemState::insertSystemStateElement(std::pair<SystemStateKey, SystemStateElementUniquePtr>&& key_ptr)
{
  assert(key_ptr.first.valueless_by_exception() == false);
  assert(key_ptr.second != nullptr);

  state_.insert_or_assign(key_ptr.first, std::move(key_ptr.second));

  utils::Logger::info("Created System State element [" + toString(key_ptr.first) + "]");
}

void SystemState::insertSystemStateElement(
    std::vector<std::pair<SystemStateKey, SystemStateElementUniquePtr>>& keys_ptrs)
{
  for (auto&& key_ptr : std::move(keys_ptrs))
  {
    insertSystemStateElement(std::move(key_ptr));
  }
}

const SE23& SystemState::T() const
{
  return std::static_pointer_cast<ExtendedPoseState>(state_.at(SystemStateElementName::T))->T_;
}

const SE3 SystemState::P() const { return SE3(T().q(), {T().p()}); }

const SE3 SystemState::V() const { return SE3(T().q(), {T().v()}); }

const Vector6& SystemState::b() const
{
  return std::static_pointer_cast<BiasState>(state_.at(SystemStateElementName::b))->b_;
}

const SE3& SystemState::S() const
{
  if (opts_.enable_camera_extrinsics_calibration_)
  {
    return std::static_pointer_cast<CameraExtrinsicState>(state_.at(SystemStateElementName::S))->S_;
  }
  else
  {
    return opts_.initial_camera_extrinsics_;
  }
}

const In& SystemState::K() const
{
  if (opts_.enable_camera_intrinsics_calibration_)
  {
    return std::static_pointer_cast<CameraIntrinsicState>(state_.at(SystemStateElementName::K))->K_;
  }
  else
  {
    return opts_.initial_camera_intrinsics_;
  }
}

const Vector3& SystemState::f(const uint& feat_id) const
{
  return std::static_pointer_cast<FeatureState>(state_.at(feat_id))->f_;
}

const Vector3 SystemState::ge3() const { return opts_.gravity_ * (Vector3() << 0, 0, -1).finished(); }

std::string SystemState::toString(const SystemStateKey& key)
{
  std::string name;
  if (std::holds_alternative<SystemStateElementName>(key))
  {
    switch (std::get<SystemStateElementName>(key))
    {
      case SystemStateElementName::T:
        name = "Extended Pose (T)";
        break;
      case SystemStateElementName::b:
        name = "Bias (b)";
        break;
      case SystemStateElementName::S:
        name = "Extrinsic Camera Calibration (S)";
        break;
      case SystemStateElementName::K:
        name = "Intrinsic camera Calibration (K)";
        break;
    }
  }
  else
  {
    name = "Persistent feature (f) with id: " + std::to_string(std::get<uint>(key));
  }
  return name;
}

}  // namespace msceqf