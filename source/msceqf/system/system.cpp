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

#include "msceqf/system/system.hpp"

namespace msceqf
{
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
}  // namespace msceqf