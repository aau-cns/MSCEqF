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
   * @param opts Tracker options
   * @param intrinsics Camera intrinsics as R4 vector (fx, fy, cx, cy)
   */
  TrackManager(const TrackManagerOptions& opts, const Vector4& intrinsics);

  /**
   * @brief Process a single camera measurement. Forward camera measurement to tracker, and update tracks
   *
   * @param cam Camera measurement
   */
  void processCamera(Camera& cam);

  /**
   * @brief Process a single features measurement. update tracks
   *
   * @param features Features measurement
   */
  void processFeatures(const TriangulatedFeatures& features);

  /**
   * @brief Get all the tracks
   *
   * @return Tracks
   */
  const Tracks& tracks() const;

  /**
   * @brief Get all the ids corresponding to active and lost tracks at a given timestamp. Active tracks are defined as
   * tracks that have are actively tracked at a given timestamp. Lost tracks are defined as tracks that are not being
   * tracked at a given timestamp and thus they do not have coordinates at a given timestamp.
   *
   * @param timestamp Timestamp
   * @param active_ids Active tracks ids
   * @param lost_ids Lost tracks ids
   */
  void tracksIds(const fp& timestamp, std::unordered_set<uint>& active_ids, std::unordered_set<uint>& lost_ids) const;

  /**
   * @brief Get all the ids corresponding to active tracks at a given timestamp. Active tracks are defined as
   * tracks that have are actively tracked at a given timestamp.
   *
   * @param timestamp Timestamp
   * @param active_ids Active tracks ids
   */
  void activeTracksIds(const fp& timestamp, std::unordered_set<uint>& active_ids) const;

  /**
   * @brief Get all the ids corresponding to lost tracks at a given timestamp. Lost tracks are defined as tracks that
   * are not being tracked at a given timestamp and thus they do not have coordinates at a given timestamp.
   *
   * @param timestamp Timestamp
   * @param lost_ids Lost tracks ids
   */
  void lostTracksIds(const fp& timestamp, std::unordered_set<uint>& lost_ids) const;

  /**
   * @brief Remove all the tracks corresponding to given ids
   *
   * @param ids Ids of tracks to be removed
   *
   * @note The use of unordered_set as a hash set improves performance compared to a std::vector<uint>
   */
  void removeTracksId(const std::unordered_set<uint>& ids);

  /**
   * @brief Remove the tail of tracks. This method remove from each track all the coordinates as well as the
   * timestamps that are older (or equal) to the given timestamp.
   *
   * @param timestamp Timestamp
   * @param remove_equal Flag to indicate whether to include in the removal also the coordinates and timestamps at the
   * given timestamp
   */
  void removeTracksTail(const fp& timestamp, const bool& remove_equal = true);

  /**
   * @brief Clear all the tracks
   *
   */
  inline void clear() { tracks_.clear(); }

  /**
   * @brief Get the camera pointer
   *
   * @return Pointer to the camera object
   */
  const PinholeCameraUniquePtr& cam() const;

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