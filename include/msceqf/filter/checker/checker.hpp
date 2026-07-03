// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#ifndef CHECKER_HPP
#define CHECKER_HPP

#include "msceqf/options/msceqf_options.hpp"
#include "types/fptypes.hpp"
#include "utils/tools.hpp"
#include "vision/track.hpp"

namespace msceqf
{
/**
 * @brief Simple class to perform various checks
 *
 */
class Checker
{
 public:
  /**
   * @brief Checker constructor
   *
   * @param opts checker options
   */
  Checker(const CheckerOptions& opts);

  /**
   * @brief Perform disparity check on given tracks
   *
   * @param tracks tracks up to date used for disparity check
   *
   * @return true if disparity check succeed (diparity above threshold), false if no disparity is detected (disparity
   * below threshold)
   *
   * @note This method checks only tracks that are as long as the first track. This ideally should avoid to use newly
   * detected/tracked features corresponding to temporary objects moving in front of the camera
   */
  [[nodiscard]] bool disparityCheck(const Tracks& tracks) const;

 private:
  CheckerOptions opts_;  //!< The checker options
};

}  // namespace msceqf

#endif  // CHECKER_HPP