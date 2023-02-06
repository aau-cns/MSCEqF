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
#include <unordered_set>

#include "types/fptypes.hpp"
#include "vision/tracker.hpp"

namespace msceqf
{

/**
 * @brief This class manages the multiple tracks of feature traked in time
 *
 */
class TrackManager
{
 public:
  /**
   * @brief TrackManager constructor
   *
   * @param opts
   * @param intrinsics
   */
  TrackManager(const TrackManagerOptions& opts, const Vector4& intrinsics);

  /**
   * @brief Process a single camera measurement. Forward camera measurement to tracker, and update tracks
   *
   * @param cam
   */
  void processCamera(Camera& cam);

  /**
   * @brief get all the tracks
   *
   * @return const Tracks&
   */
  const Tracks& tracks() const;

  /**
   * @brief Get all the active and lost tracks at a given timestamp. Active tracks are defined as tracks that have are
   * actively tracked at a given timestamp. Lost tracks are defined as tracks that are not being tracked at a given
   * timestamp and thus they do not have coordinates at a given timestamp.
   *
   * @param timestamp
   * @param active
   * @param lost
   */
  void tracksAt(const fp& timestamp, Tracks& active, Tracks& lost) const;

  /**
   * @brief Get all the active tracks at a given timestamp. Active tracks are defined as tracks that have are
   * actively tracked at a given timestamp.
   *
   * @param timestamp
   * @param active
   */
  void activeTracksAt(const fp& timestamp, Tracks& active) const;

  /**
   * @brief Remove all the tracks corresponding to given ids
   *
   * @param ids
   *
   * @note The use of unordered_set as a hash set improves performance compared to a std::vector<uint>
   */
  void removeTracksId(const std::unordered_set<uint>& ids);

  /**
   * @brief Remove the tail of tracks. This method remove from each track all the coordinates as well as the timestamps
   * that are older or equal to the given timestamp.
   *
   * @param timestamp
   */
  void removeTracksTail(const fp& timestamp);

 private:
  /**
   * @brief Updeate tracks with current features from tracker
   *
   */
  void updateTracks();

  Tracker tracker_;  //!< Feature tracker
  Tracks tracks_;    //!< Tracks

  size_t max_track_length_;  //!< Maximum length of a single track
};

}  // namespace msceqf

#endif  // TRACK_MANAGER_HPP