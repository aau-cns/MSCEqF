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

#include "vision/tracker.hpp"

#include "utils/logger.hpp"
#include "utils/tools.hpp"

namespace msceqf
{

Tracker::Tracker(const TrackerOptions& opts, const Vector4& intrinsics)
    : opts_(opts)
    , cam_()
    , detector_()
    , max_kpts_per_cell_(opts_.max_features_ / (opts_.grid_x_size_ * opts_.grid_y_size_ * opts_.pyramid_levels_))
    , id_(0)
    , previous_pyramids_()
    , previous_mask_()
    , previous_features_()
    , current_pyramids_()
    , current_features_()
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
  assert(!cam.image_.empty());

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
  // Assign timestamp
  current_features_.first = cam.timestamp_;

  // Update opts_.pyramid_levels_ with the actual number of pyramid levels
  // (opts_.pyramid_levels_ - 1) is given since maxLevel is 0-based in buildOpticalFlowPyramid
  opts_.pyramid_levels_ =
      cv::buildOpticalFlowPyramid(cam.image_, current_pyramids_, win_, opts_.pyramid_levels_ - 1) + 1;

  if (previous_features_.second.empty())
  {
    detectAndUndistort(current_pyramids_, cam.mask_, current_features_.second);
  }
  else
  {
    std::vector<uchar> klt_mask;
    std::vector<uchar> ransac_mask;

    detectAndUndistort(previous_pyramids_, previous_mask_, previous_features_.second);
    matchKLT(klt_mask);
    ransac(ransac_mask);

    // Check if there are invalid features
    assert(klt_mask.size() == ransac_mask.size());
    std::vector<bool> invalid(klt_mask.size());
    bool found_invalid = false;
    for (size_t i = 0; i < invalid.size(); i++)
    {
      auto& uv = current_features_.second.uvs_[i];
      found_invalid |= (invalid[i] = !klt_mask[i] || !ransac_mask[i] || uv.x < 0 || uv.y < 0 ||
                                     uv.x > current_pyramids_[0].cols || uv.y > current_pyramids_[0].rows);
    }

    // Remove invalid features (coordinates and ids)
    if (found_invalid)
    {
      current_features_.second.removeInvalid(invalid);
    }
  }

  // Swap pyramids and mask
  previous_pyramids_.swap(current_pyramids_);
  cv::swap(previous_mask_, cam.mask_);

  // Copy features
  previous_features_ = current_features_;
}

void Tracker::detectAndUndistort(std::vector<cv::Mat>& pyramids, cv::Mat& mask, Features& features)
{
  // Return if we have enough features
  if (features.size() > opts_.min_features_)
  {
    return;
  }

  // Compute keypoints needed (The amount of keypoints we need to extract)
  uint needed_kpts = opts_.max_features_ - features.size();

  utils::Logger::debug("Not enough existing features, detecting new features");

  // Declare detected features (through all the pyramid levels)
  std::vector<FeaturesCoordinates> detected(opts_.pyramid_levels_);

  // Declare cell keypoints
  std::vector<Keypoints> cell_kpts(opts_.grid_x_size_ * opts_.grid_y_size_);

  // Mask existing features
  maskGivenFeatures(mask, features.uvs_);

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
        std::max(uint(1), needed_kpts / (opts_.grid_x_size_ * opts_.grid_y_size_ * opts_.pyramid_levels_));

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
                            max_kpts_per_cell_ = ((needed_kpts / opts_.pyramid_levels_) - num_detected.load()) /
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

    // Flatten cell keypoints (convert to features)
    for (const auto& kpts : cell_kpts)
    {
      for (const auto& kpt : kpts)
      {
        detected[i].emplace_back(kpt.pt);
      }
    }

    // mask already detected features
    maskGivenFeatures(mask, detected[i]);
  }

  // Flatten detected features
  FeaturesCoordinates detected_flat = utils::flatten(detected);

  // Return if no keypoints has been found
  if (detected_flat.empty())
  {
    utils::Logger::debug("Failed to detect new feature");
    return;
  }

  // // Remove features below minimum distance
  // if (opts_.min_px_dist_ > 0)
  // {
  //   removeCloseFeatures(detected_flat);
  // }

  // Sub-pixel refinement for all the detected keypoints
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.001);
  cv::cornerSubPix(pyramids[0], detected_flat, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  // Undistort
  cam_->undistort(detected_flat);

  // Normalize
  FeaturesCoordinates normalized_detected_flat(detected_flat);
  cam_->normalize(normalized_detected_flat);

  size_t detected_size = detected_flat.size();
  size_t total_size = features.size() + detected_size;

  features.uvs_.reserve(total_size);
  features.normalized_uvs_.reserve(total_size);

  // Append newly detected features to existing
  features.uvs_.insert(features.uvs_.end(), std::make_move_iterator(detected_flat.begin()),
                       std::make_move_iterator(detected_flat.end()));
  features.normalized_uvs_.insert(features.normalized_uvs_.end(),
                                  std::make_move_iterator(normalized_detected_flat.begin()),
                                  std::make_move_iterator(normalized_detected_flat.end()));

  // Assign id to newly detected features
  features.ids_.reserve(total_size);
  uint end_id_ = id_ + detected_size;
  for (; id_ < end_id_; ++id_)
  {
    features.ids_.emplace_back(id_);
  }
}

void Tracker::maskGivenFeatures(cv::Mat& mask, const FeaturesCoordinates& points)
{
  // Return if we have no points
  if (points.empty())
  {
    return;
  }

  int px_dist = std::ceil(opts_.min_px_dist_ / 2);
  for (const auto& point : points)
  {
    int x = static_cast<int>(point.x);
    int y = static_cast<int>(point.y);
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
  cell_kpts.resize(std::min(cell_kpts.size(), (max_kpts_per_cell_.load() - previous_features_.second.size())));
}

void Tracker::matchKLT(std::vector<uchar>& mask)
{
  // Set current features to previous for initial flow
  current_features_.second = previous_features_.second;

  std::vector<float> error;
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

  cv::calcOpticalFlowPyrLK(previous_pyramids_, current_pyramids_, previous_features_.second.uvs_,
                           current_features_.second.uvs_, mask, error, win_, opts_.pyramid_levels_ - 1, criteria,
                           cv::OPTFLOW_USE_INITIAL_FLOW);

  // Normalized tracked features
  FeaturesCoordinates normalized_tracked(current_features_.second.uvs_);
  cam_->normalize(normalized_tracked);
  current_features_.second.normalized_uvs_ = std::move(normalized_tracked);
}

void Tracker::ransac(std::vector<uchar>& mask)
{
  // Check to have enough points otherwise return
  if (previous_features_.second.size() < 10)
  {
    mask.resize(previous_features_.second.size());
    std::fill(mask.begin(), mask.end(), uchar(0));
    return;
  }

  cv::findFundamentalMat(previous_features_.second.normalized_uvs_, current_features_.second.normalized_uvs_,
                         cv::FM_RANSAC, 0.25, 0.999, mask);
}

const Tracker::TimedFeatures& Tracker::currentFeatures() const { return current_features_; }

}  // namespace msceqf