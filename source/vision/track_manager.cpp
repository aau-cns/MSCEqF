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

#include "vision/track_manager.hpp"

#include "utils/logger.hpp"

namespace msceqf
{

TrackManager::TrackManager(const TrackManagerOptions& opts, const Vector4& intrinsics)
    : tracker_(opts.tracker_options_, intrinsics), tracks_(), max_track_length_(opts.max_track_length_)
{
}

void TrackManager::processCamera(Camera& cam)
{
  tracker_.processCamera(cam);
  updateTracks();
}

const Tracks& TrackManager::tracks() const { return tracks_; }

void TrackManager::tracksAt(const fp& timestamp, Tracks& active, Tracks& lost) const
{
  for (const auto& [id, track] : tracks_)
  {
    if (std::find(track.timestamps_.begin(), track.timestamps_.end(), timestamp) != track.timestamps_.end())
    {
      active.insert_or_assign(id, track);
    }
    else
    {
      lost.insert_or_assign(id, track);
    }
  }
}

void TrackManager::activeTracksAt(const fp& timestamp, Tracks& active) const
{
  for (const auto& [id, track] : tracks_)
  {
    if (std::find(track.timestamps_.begin(), track.timestamps_.end(), timestamp) != track.timestamps_.end())
    {
      active.insert_or_assign(id, track);
    }
  }
}

void TrackManager::removeTracksId(const std::unordered_set<uint>& ids)
{
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    if (ids.count(it->first))
    {
      it = tracks_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void TrackManager::removeTracksTail(const fp& timestamp)
{
  for (auto& [id, track] : tracks_)
  {
    track.removeTail(timestamp);
  }
}

void TrackManager::updateTracks()
{
  auto& current_features = tracker_.currentFeatures();

  if (current_features.second.empty())
  {
    utils::Logger::warn("Impossible to update tracks. No features has been detected/tracked");
    return;
  }

  // for each feature/id either initialize a new track or update the existing track associated to the id
  for (size_t i = 0; i < current_features.second.size(); ++i)
  {
    auto& uv = current_features.second.uvs_[i];
    auto& uvn = current_features.second.normalized_uvs_[i];
    auto& id = current_features.second.ids_[i];

    // Initialize new element into tracks or extend existing track
    auto& track_ref = tracks_.try_emplace(id, Track()).first->second;
    track_ref.uvs_.emplace_back(uv);
    track_ref.normalized_uvs_.emplace_back(uvn);
    track_ref.timestamps_.emplace_back(current_features.first);

    // Remove tracks that are too long (Keep memory bounded)
    if (track_ref.size() > max_track_length_)
    {
      utils::Logger::warn("Max track (id: " + std::to_string(id) + ") length reached, removing track tail");
      track_ref.uvs_.erase(track_ref.uvs_.begin());
      track_ref.normalized_uvs_.erase(track_ref.normalized_uvs_.begin());
      track_ref.timestamps_.erase(track_ref.timestamps_.begin());
    }
  }
}

}  // namespace msceqf
