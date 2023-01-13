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

namespace msceqf
{

SystemState::SystemState(const SystemState& other) : state_()
{
  // Copy state_
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }
}

SystemState::SystemState(SystemState&& other) noexcept : state_(std::move(other.state_)) {}

SystemState& SystemState::operator=(const SystemState& other)
{
  // Copy state_
  state_.clear();
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }

  return *this;
}

SystemState& SystemState::operator=(SystemState&& other) noexcept
{
  state_ = std::move(other.state_);
  return *this;
}

SystemState ::~SystemState() { state_.clear(); }

void SystemState::insertSystemStateElement(std::pair<SystemStateKey, SystemStateElementUniquePtr>&& key_ptr)
{
  assert(key_ptr.first.valueless_by_exception() == false);
  assert(key_ptr.second != nullptr);

  // Insertion
  state_.try_emplace(key_ptr.first, std::move(key_ptr.second));
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

const Vector6& SystemState::b() const
{
  return std::static_pointer_cast<BiasState>(state_.at(SystemStateElementName::b))->b_;
}

const SE3& SystemState::S() const
{
  return std::static_pointer_cast<CameraExtrinsicState>(state_.at(SystemStateElementName::S))->S_;
}

const In& SystemState::K() const
{
  return std::static_pointer_cast<CameraIntrinsicState>(state_.at(SystemStateElementName::K))->K_;
}

const Vector3& SystemState::f(const uint& feat_id) const
{
  return std::static_pointer_cast<FeatureState>(state_.at(feat_id))->f_;
}

}  // namespace msceqf