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

#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <opencv2/opencv.hpp>

#include "sensors/sensor_data.hpp"
#include "vision/track_manager.hpp"

namespace msceqf
{
class Visualizer
{
 public:
  /**
   * @brief Construct a new Visualizer object
   *
   * @param track_manager
   */
  Visualizer(const TrackManager& track_manager) : track_manager_(track_manager), colors_(), delay_(1)
  {
    colors_.emplace(0, cv::Scalar(38, 0, 165));     // red
    colors_.emplace(1, cv::Scalar(39, 48, 215));    // dark red
    colors_.emplace(2, cv::Scalar(67, 109, 244));   // orange
    colors_.emplace(3, cv::Scalar(97, 174, 253));   // light orange
    colors_.emplace(4, cv::Scalar(139, 224, 254));  // yellow
    colors_.emplace(5, cv::Scalar(191, 255, 255));  // light yellow
    colors_.emplace(6, cv::Scalar(139, 239, 217));  // light green
    colors_.emplace(7, cv::Scalar(106, 217, 166));  // green
    colors_.emplace(8, cv::Scalar(99, 189, 102));   // slightly darker green
    colors_.emplace(9, cv::Scalar(80, 152, 26));    // dark green
    colors_.emplace(10, cv::Scalar(55, 104, 0));    // very dark green
  }

  /**
   * @brief Camera image with overlayed tracks
   *
   * @param cam
   */
  cv::Mat3b imageWithTracks(const Camera& cam, const std::string& text = "") const
  {
    cv::Mat undistorted_image;
    track_manager_.cam()->undistortImage(cam.image_, undistorted_image);

    cv::Mat3b color_image = undistorted_image;
    if (undistorted_image.channels() == 1)
    {
      cv::cvtColor(undistorted_image, color_image, cv::COLOR_GRAY2BGR, 3);
    }

    cv::Mat mask;
    track_manager_.cam()->undistortImage(cam.mask_, mask);
    mask = cv::Scalar(255) - mask;

    std::vector<cv::Mat> channels;
    cv::split(color_image, channels);
    channels[2] = channels[2] + mask;
    cv::merge(channels, color_image);

    if (text.compare("") != 0)
    {
      cv::putText(color_image, text, cv::Point(50, 50), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 0, 0), 2);
    }

    Tracker::Keypoints active_kpts;
    std::unordered_set<uint> active_ids;

    track_manager_.activeTracksIds(cam.timestamp_, active_ids);
    if (!active_ids.empty())
    {
      const auto& tracks = track_manager_.tracks();
      for (auto& id : active_ids)
      {
        const auto& color = colors_.at(id % colors_.size());
        for (size_t i = 0; i < tracks.at(id).size() - 1; ++i)
        {
          cv::line(color_image, tracks.at(id).uvs_.at(i), tracks.at(id).uvs_.at(i + 1), color, 1);
        }
        cv::circle(color_image, tracks.at(id).uvs_.back(), 3, color, -1);
      }
    }

    return color_image;
  }

  /**
   * @brief Visualize imge with history of tracks
   *
   * @param cam
   */
  void visualizeImageWithTracks(const Camera& cam, const std::string& text = "") const
  {
    cv::imshow("Undistorted image with tracks", imageWithTracks(cam, text));
    cv::waitKey(delay_);
  }

 private:
  const TrackManager& track_manager_;            //!< track manager
  std::unordered_map<uint, cv::Scalar> colors_;  //!< colors for tracks (BGR)
  int delay_;                                    //!< delay for visualization
};
}  // namespace msceqf

#endif  // VISUALIZER_HPP
