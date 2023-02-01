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

#include "msceqf/vision/tracker.hpp"

#include "utils/tools.hpp"

namespace msceqf
{

Tracker::Tracker(const TrackerOptions& opts, const Vector4& intrinsics)
    : opts_(opts)
    , cam_()
    , detector_()
    , max_kpts_per_cell_(opts_.max_features_ / (opts_.grid_x_size_ * opts_.grid_y_size_ * opts_.pyramid_levels_))
    , previous_pyramids_()
    , previous_mask_()
    , previous_kpts_()
    , current_pyramids_()
    , current_kpts()
    , matches_()
    , win_(cv::Size(opts_.optical_flow_win_size_, opts_.optical_flow_win_size_))
{
  assert(opts_.pyramid_levels_ > 0);

  previous_pyramids_.reserve(opts_.pyramid_levels_);
  current_pyramids_.reserve(opts_.pyramid_levels_);

  switch (opts_.distortion_model_)
  {
    case DistortionModel::RADTAN:
    {
      cam_ = createCamera<RadtanCamera>(opts_.cam_options_, intrinsics);
      utils::Logger::info("Initialized KLT tracker based on pinhole-radtan camera model");
    }
    break;
    default:
      break;
  }

  switch (opts_.detector_)
  {
    case FeatureDetector::FAST:
      detector_ = cv::FastFeatureDetector::create(opts_.fast_opts_.fast_threshold_);
      utils::Logger::info("Initialized KLT tracker FAST feature detector");
      break;
    case FeatureDetector::GFTT:
      detector_ = cv::GFTTDetector::create(opts_.max_features_, opts_.gftt_opts_.quality_level_, opts_.min_px_dist_);
      utils::Logger::info("Initialized KLT tracker Shi-Tomasi feature detector");
      break;
    default:
      break;
  }

  if (max_kpts_per_cell_ < 1)
  {
    utils::Logger::warn("Number of cells grater than max features, extracting one feature per cell.");
    max_kpts_per_cell_ = 1;
  }

  assert(cam_ != nullptr);
  assert(!detector_.empty());
}

void Tracker::processCamera(Camera& cam)
{
  if (cam.image_.channels() > 1)
  {
    cv::cvtColor(cam.image_, cam.image_, cv::COLOR_BGR2GRAY);
  }

  switch (opts_.equalizer_)
  {
    case EqualizationMethod::NONE:
      break;
    case EqualizationMethod::HISTOGRAM:
      cv::equalizeHist(cam.image_, cam.image_);
      break;
    case EqualizationMethod::CLAHE:
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
      clahe->apply(cam.image_, cam.image_);
      break;
  }

  track(cam);
}

void Tracker::track(Camera& cam)
{
  // Update opts_.pyramid_levels_ with the actual number of pyramid levels
  // (opts_.pyramid_levels_ - 1) is given since maxLevel is 0-based in buildOpticalFlowPyramid
  opts_.pyramid_levels_ =
      cv::buildOpticalFlowPyramid(cam.image_, current_pyramids_, win_, opts_.pyramid_levels_ - 1) + 1;

  // If we have no previous keypoints, just detect new features and undistort them
  if (previous_kpts_.empty())
  {
    // utils::Logger::debug("No existing keypoints, detecting new keypoints");
    detectAndUndistort(current_pyramids_, cam.mask_, current_kpts);
  }

  // // Track keypoints temporally

  // if (previous_kpts_.size() < opts_.min_features_)
  // {
  //   // utils::Logger::debug("keypoint below minimum value, detecting new keypoints");
  //   Keypoints new_previous_kpts;
  //   detectAndUndistort(previous_camera_meas_, new_previous_kpts);
  //   undistortFeatures(new_previous_kpts);
  //   // Track new_current_kpts from new_previous_kpts
  //   // merge new_current_kpts tracked with current_kpts
  // }

  // undistortFeatures(current_kpts);
  // //
  // // END
  // //

  // // matches_ = ???;

  // TEST
  cv::drawKeypoints(cam.image_, current_kpts, cam.image_, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow("Image with keypoints", cam.image_);
  cv::waitKey();

  // // Should i swap them? clear?
  previous_pyramids_ = current_pyramids_;
  previous_mask_ = cam.mask_;
  previous_kpts_ = current_kpts;
}

void Tracker::detectAndUndistort(std::vector<cv::Mat>& pyramids, cv::Mat& mask, Keypoints& current_kpts)
{
  // Declare detected keypoints (through all the pyramid levels)
  std::vector<std::vector<Feature::FeatureCoordinates>> detected(opts_.pyramid_levels_);

  // Mask existing keypoints
  maskPreviouskeypoints(mask);

  // Declare cell keypoints
  std::vector<Keypoints> cell_kpts(opts_.grid_x_size_ * opts_.grid_y_size_);

  for (uint i = 0; i < opts_.pyramid_levels_; ++i)
  {
    uint pyr_idx = 2 * i;
    int scale = utils::pow2(i);

    int cell_width = pyramids[pyr_idx].cols / opts_.grid_x_size_;
    int cell_height = pyramids[pyr_idx].rows / opts_.grid_y_size_;

    // Downsample mask
    cv::Mat resized_mask = mask.clone();
    for (uint s = 0; s < i; ++s)
    {
      cv::pyrDown(resized_mask, resized_mask, cv::Size(resized_mask.cols / 2.0, resized_mask.rows / 2.0));
    }

    // Reset value of max keypoints per cell for the given pyramid
    max_kpts_per_cell_ =
        std::max(uint(1), opts_.max_features_ / (opts_.grid_x_size_ * opts_.grid_y_size_ * opts_.pyramid_levels_));

    std::atomic<size_t> num_detected(0);
    std::atomic<int> cell_cnt(0);

    // Parallel feature extraction for each cell of the grid.
    // Re-computation of max_kpts_per_cell_ based on how many feature have been extracted in previous cells.
    cv::parallel_for_(cv::Range(0, opts_.grid_x_size_ * opts_.grid_y_size_),
                      [&](const cv::Range& range)
                      {
                        for (int cell_idx = range.start; cell_idx < range.end; ++cell_idx)
                        {
                          cell_kpts[cell_idx].clear();

                          int y = cell_idx / opts_.grid_x_size_;
                          int x = cell_idx % opts_.grid_x_size_;
                          cv::Rect cell(x * cell_width, y * cell_height, cell_width, cell_height);

                          extractCellKeypoints(pyramids[pyr_idx](cell), resized_mask(cell), cell_kpts[cell_idx]);

                          // Dynamic max keypoints per cell based on the amount of previously extracted keypoints
                          num_detected += cell_kpts[cell_idx].size();
                          if (max_kpts_per_cell_.load() > 1)
                          {
                            max_kpts_per_cell_ = ((opts_.max_features_ / opts_.pyramid_levels_) - num_detected.load()) /
                                                 ((opts_.grid_x_size_ * opts_.grid_y_size_) - cell_cnt.load());
                          }

                          ++cell_cnt;

                          // Shift keypoints based on cell and scale
                          if (x > 0 || y > 0 || scale > 1)
                          {
                            std::for_each(cell_kpts[cell_idx].begin(), cell_kpts[cell_idx].end(),
                                          [&x, &y, &cell_width, &cell_height, &scale](cv::KeyPoint& kpt)
                                          {
                                            kpt.pt.x = (kpt.pt.x + (x * cell_width)) * scale;
                                            kpt.pt.y = (kpt.pt.y + (y * cell_height)) * scale;
                                          });
                          }
                        }
                      });

    // Flatten cell keypoints (convert to FeatureCoordinates vector)
    for (const auto& kpts : cell_kpts)
    {
      for (const auto& kpt : kpts)
      {
        detected[i].emplace_back(kpt.pt);
      }
    }

    // mask already detected keypoints
    maskGivenkeypoints(mask, detected[i]);
  }

  // Flatten detected keypoints
  std::vector<Feature::FeatureCoordinates> detected_flat = utils::flatten(detected);

  // // Remove features below minimum distance
  // if (opts_.min_px_dist_ > 0)
  // {
  //   removeCloseFeatures(detected);
  // }

  // Sub-pixel refinement for all the detected keypoints
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.001);
  cv::cornerSubPix(pyramids[0], detected_flat, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  // Undistort
  cam_->undistort(detected_flat);

  // Assign
  current_kpts.reserve(detected_flat.size());
  for (const auto& p : detected_flat)
  {
    current_kpts.emplace_back(p.x, p.y, 1.0f);
  }
}

void Tracker::maskPreviouskeypoints(cv::Mat& mask)
{
  int px_dist = std::ceil(opts_.min_px_dist_ / 2);
  for (const auto& kpt : previous_kpts_)
  {
    int x = static_cast<int>(kpt.pt.x);
    int y = static_cast<int>(kpt.pt.y);
    int x1 = std::max(0, x - px_dist);
    int y1 = std::max(0, y - px_dist);
    int x2 = std::min(mask.cols - 1, x + px_dist);
    int y2 = std::min(mask.rows - 1, y + px_dist);
    mask(cv::Rect(x1, y1, x2 - x1 + 1, y2 - y1 + 1)) = 0;
  }
}

void Tracker::extractCellKeypoints(const cv::Mat& cell, const cv::Mat& mask, Keypoints& cell_kpts)
{
  detector_->detect(cell, cell_kpts, mask);

  // // Remove features below minimum distance in this cell if we are using FAST
  // if (opts_.min_px_dist_ > 0 && opts_.detector_ == FeatureDetector::FAST)
  // {
  //   removeCloseFeatures(cell_kpts);
  // }

  // Sort detected keypoints based on fast score
  std::sort(cell_kpts.begin(), cell_kpts.end(),
            [](const cv::KeyPoint& pre, const cv::KeyPoint& post) { return pre.response > post.response; });

  // Cap keypoints to max_kpts_per_cell_, keeping the ones with the highest score
  assert((max_kpts_per_cell_.load() - previous_kpts_.size()) > 0);
  cell_kpts.resize(std::min(cell_kpts.size(), (max_kpts_per_cell_.load() - previous_kpts_.size())));
}

// void Tracker::match()
// {
//   cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
//   cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0, pts1, mask_klt, error, win_, pyr_levels, criteria,
//                            cv::OPTFLOW_USE_INITIAL_FLOW);
// }

}  // namespace msceqf