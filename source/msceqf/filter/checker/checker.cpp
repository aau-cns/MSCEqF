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