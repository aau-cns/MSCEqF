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