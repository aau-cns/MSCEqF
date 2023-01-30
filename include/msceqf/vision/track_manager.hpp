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

#ifndef TRACK_MANAGER_HPP
#define TRACK_MANAGER_HPP

#include <opencv2/opencv.hpp>

#include "msceqf/vision/track.hpp"
#include "types/fptypes.hpp"

namespace msceqf
{

/**
 * @brief This class manages the multiple tracks
 *
 */
class TrackManager
{
 public:
  using Tracks = std::unordered_map<uint, Track>;  //!< Tracks defined as a id-track map

 private:
  Tracks tracks_;  //!< features tracks
};

}  // namespace msceqf

#endif  // TRACK_MANAGER_HPP