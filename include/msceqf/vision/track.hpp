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

#ifndef TRACK_HPP
#define TRACK_HPP

#include "msceqf/vision/feature.hpp"
#include "types/fptypes.hpp"
#include "utils/logger.hpp"

namespace msceqf
{

/**
 * @brief This class represent a single track.
 * A track is defined as a collection of the same features tracked temporally.
 *
 */
class Track
{
 public:
  template <typename... Args>
  Track(const fp& anchor_timestamp, Args&&... args) : anchor_timestamp_(anchor_timestamp), track_()
  {
    insertFeature(std::forward<decltype(args)>(args)...);
  }

  /**
   * @brief Insert a new feature in the features track
   *
   * @tparam Args
   * @param args
   *
   * @note A id check should be done prior calling this method
   */
  template <typename... Args>
  void insertFeature(Args&&... args)
  {
    if constexpr (std::is_constructible_v<Feature, decltype(args)...>)
    {
      track_.emplace_back(args...);
    }
    else
    {
      utils::Logger::err("Wrong arguments given when trying to insert a new feature in the features track");
    }
  }

  /**
   * @brief Get the actual track
   *
   * @return const std::vector<Feature>&
   */
  const std::vector<Feature>& getTrack() const;

  /**
   * @brief Get the timestamp of the anchor camera measurement
   *
   * @return const fp&
   */
  const fp& getAnchorTimestamp() const;

 private:
  std::vector<Feature> track_;  //<! Feature track
  fp anchor_timestamp_ = -1;    //<! timestamp of the anchor camera measurement
};

}  // namespace msceqf

#endif  // TRACK_HPP