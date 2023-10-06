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

#include "msceqf/filter/checker/checker.hpp"
#include "utils/logger.hpp"

namespace msceqf
{
Checker::Checker(const CheckerOptions& opts) : opts_(opts) {}

bool Checker::disparityCheck(const Tracks& tracks) const
{
  const auto& longest_track = std::max_element(tracks.begin(), tracks.end(), [](const auto& pre, const auto& post) {
                                return pre.second.timestamps_.size() < post.second.timestamps_.size();
                              })->second;

  const fp& longest_track_time = longest_track.timestamps_.back() - longest_track.timestamps_.front();

  if (longest_track_time < opts_.disparity_window_)
  {
    utils::Logger::info("feature tracks not long enough for disparity check in static initializer");
    return false;
  }

  fp average_disparity = 0;
  int track_cnt = 0;

  for (const auto& [id, track] : tracks)
  {
    if (track.size() == longest_track.size())
    {
      average_disparity += cv::norm(track.uvs_.back() - track.uvs_.front());
      ++track_cnt;
    }
  }
  average_disparity /= track_cnt;

  return average_disparity > opts_.disparity_threshold_ ? true : false;
}
}  // namespace msceqf