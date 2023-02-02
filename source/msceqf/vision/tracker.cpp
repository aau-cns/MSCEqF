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
  if (previous_features_.empty())
  {
    utils::Logger::debug("No existing keypoints, detecting new keypoints");
    // detectAndUndistort(current_pyramids_, cam.mask_, current_features_, true);
    detectAndUndistort(current_pyramids_, cam.mask_, current_features_);
  }
  else
  {
    std::vector<uchar> klt_mask;

    // Match keypoints (track keypoints temporally with Optical flow)
    matchKLT(previous_features_, current_features_, klt_mask);

    // [TODO] Can I avoid this by always calling detect and undistort but check inside if i need features?
    if (previous_features_.size() < opts_.min_features_)
    {
      std::vector<uchar> new_klt_mask;
      FeaturesPoints new_previous_feats, new_current_feats;

      utils::Logger::debug("keypoint below minimum value, detecting new keypoints");
      detectAndUndistort(previous_pyramids_, previous_mask_, new_previous_feats);
      matchKLT(new_previous_feats, new_current_feats, new_klt_mask);

      // Merge new_current_kpts tracked with current_features_
      current_features_.insert(current_features_.end(), std::make_move_iterator(new_current_feats.begin()),
                               std::make_move_iterator(new_current_feats.end()));

      // Merge new_klt_mask with klt_mask
      klt_mask.insert(klt_mask.end(), std::make_move_iterator(new_klt_mask.begin()),
                      std::make_move_iterator(new_klt_mask.end()));
    }

    // [TODO] IF NO NEW DETECTED current_features_ would have the same ids as previous_features_
    // [TODO] How to keep track of the id when removing invalid??
    // [TODO] IF THERE ARE NEW DETCTED a subset of current_features_ would have the same ids as previous_features_,
    // while another subset would have new ids
    // [TODO] How to keep track of the id when removing invalid??
    // [TODO] I probably need a different data structure for this, something that relates features points as well as ids

    // Normalize features (improve the conditioning of the problem)
    // cam_->normalize(current_features_);

    std::vector<uchar> ransac_mask;

    // [TODO] Should i check to have at least 8 features?
    // Reject outlier with RANSAC
    cv::findFundamentalMat(previous_features_, current_features_, cv::FM_RANSAC, 0.25, 0.999, ransac_mask);

    assert(klt_mask.size() == ransac_mask.size());

    // Remove invalid features
    std::vector<bool> valid(klt_mask.size());
    for (size_t i = 0; i < valid.size(); i++)
    {
      valid[i] = klt_mask[i] && ransac_mask[i] && current_features_[i].x > 0 && current_features_[i].y > 0 &&
                 current_features_[i].x < current_pyramids_[0].cols &&
                 current_features_[i].y < current_pyramids_[0].rows;
    }
    current_features_.erase(
        std::remove_if(current_features_.begin(), current_features_.end(),
                       [&valid, this](const cv::Point2f& feat) { return !valid[&feat - &current_features_[0]]; }),
        current_features_.end());
  }

  // [TODO] Create Feature
  // timestamp: from cam meas
  // uv: denormalize normalized uv
  // uvn: current_features_
  // id: ???
  // [TODO] need to think what i need. Do i need matches? or a feature database?

  // TEST
  Keypoints kpts;
  for (auto feat : current_features_)
  {
    // cam_->denormalize(feat);
    kpts.emplace_back(feat.x, feat.y, 5.0f);
  }
  cv::Mat tmp;
  cv::drawKeypoints(cam.image_, kpts, tmp, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow("Image with keypoints", tmp);
  cv::waitKey(1);

  // Swap pyramids and mask
  previous_pyramids_.swap(current_pyramids_);
  cv::swap(previous_mask_, cam.mask_);

  // Copy keypoints
  previous_features_.assign(current_features_.begin(), current_features_.end());
}

void Tracker::detectAndUndistort(std::vector<cv::Mat>& pyramids,
                                 cv::Mat& mask,
                                 FeaturesPoints& current_features,
                                 const bool& normalize)
{
  // Declare detected features (through all the pyramid levels)
  std::vector<FeaturesPoints> detected(opts_.pyramid_levels_);

  // Mask existing features
  maskGivenFeatures(mask, previous_features_);

  // Declare cell keypoints
  std::vector<Keypoints> cell_kpts(opts_.grid_x_size_ * opts_.grid_y_size_);

  // Compute keypoints needed
  uint needed_kpts = opts_.max_features_ - previous_features_.size();

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
  current_features = utils::flatten(detected);

  // // Remove features below minimum distance
  // if (opts_.min_px_dist_ > 0)
  // {
  //   removeCloseFeatures(detected_flat);
  // }

  // [TODO] Return if there are no features <-- manage this situation

  // Sub-pixel refinement for all the detected keypoints
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.001);
  cv::cornerSubPix(pyramids[0], current_features, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  // Undistort
  cam_->undistort(current_features, normalize);
}

void Tracker::maskGivenFeatures(cv::Mat& mask, const FeaturesPoints& points)
{
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
  assert((max_kpts_per_cell_.load() - previous_features_.size()) > 0);
  cell_kpts.resize(std::min(cell_kpts.size(), (max_kpts_per_cell_.load() - previous_features_.size())));
}

void Tracker::matchKLT(FeaturesPoints& previous_features, FeaturesPoints& current_features, std::vector<uchar>& status)
{
  assert(previous_features.size() == current_features.size());

  std::vector<float> error;
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
  cv::calcOpticalFlowPyrLK(previous_pyramids_, current_pyramids_, previous_features, current_features, status, error,
                           win_, opts_.pyramid_levels_ - 1, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
}

}  // namespace msceqf