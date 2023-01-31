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
    , previous_camera_meas_()
    , previous_kpts_()
    , matches_()
    , max_kpts_per_cell_(opts_.max_features_ / (opts_.grid_x_size_ * opts_.grid_y_size_))
    , pyramids_(std::vector<cv::Mat>(opts_.optical_flow_pyramid_levels_))
    , win_(cv::Size(opts_.optical_flow_win_size_, opts_.optical_flow_win_size_))
{
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
  cv::buildOpticalFlowPyramid(cam.image_, pyramids_, win_, opts_.optical_flow_pyramid_levels_);

  Keypoints current_kpts;

  if (previous_kpts_.size() < opts_.min_features_)
  {
    detect(cam);
  }

  // Continue here...
  // When do i undistort?
}

void Tracker::detect(Camera& cam)
{
  int cell_width = cam.image_.cols / opts_.grid_x_size_;
  int cell_height = cam.image_.rows / opts_.grid_y_size_;

  std::vector<Keypoints> cell_kpts(opts_.grid_x_size_ * opts_.grid_y_size_);

  maskPreviouskeypoints(cam.mask_);

  // Parallel feature extraction for each cell of the grid.
  // Re-computation of max_kpts_per_cell_ based on how many feature have been extracted in previous cells.
  std::atomic<size_t> num_detected(0);
  std::atomic<int> cell_cnt(0);
  cv::parallel_for_(cv::Range(0, opts_.grid_x_size_ * opts_.grid_y_size_),
                    [&](const cv::Range& range)
                    {
                      for (int cell_idx = range.start; cell_idx < range.end; ++cell_idx)
                      {
                        int y = cell_idx / opts_.grid_x_size_;
                        int x = cell_idx % opts_.grid_x_size_;
                        cv::Rect cell(x * cell_width, y * cell_height, cell_width, cell_height);
                        extractCellKeypoints(cam.image_(cell), cam.mask_(cell), cell_kpts[cell_idx]);
                        num_detected += cell_kpts[cell_idx].size();
                        max_kpts_per_cell_ = (opts_.max_features_ - num_detected.load()) /
                                             ((opts_.grid_x_size_ * opts_.grid_y_size_) - cell_cnt.load());
                        ++cell_cnt;
                        if (x > 0 || y > 0)
                        {
                          std::for_each(cell_kpts[cell_idx].begin(), cell_kpts[cell_idx].end(),
                                        [&x, &y, &cell_width, &cell_height](cv::KeyPoint& kpt)
                                        {
                                          kpt.pt.x += (x * cell_width);
                                          kpt.pt.y += (y * cell_height);
                                        });
                        }
                      }
                    });

  // Flatten cell keypoints (convert to FeatureCoordinates vector)
  size_t size = std::accumulate(cell_kpts.begin(), cell_kpts.end(), 0,
                                [](size_t size, const Keypoints& kpts) { return size + kpts.size(); });
  std::vector<Feature::FeatureCoordinates> detected(size);
  for (const auto& kpts : cell_kpts)
  {
    for (const auto& kpt : kpts)
    {
      detected.emplace_back(kpt.pt);
    }
  }

  // Sub-pixel refinement for all the detected keypoints
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.001);
  cv::cornerSubPix(cam.image_, detected, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  // DO I NEED TO RE-CONVERT AND RETURN SOMETHING ?
  // CAN I SIMPLY APPEND TO previous_kpts_ (do i need previous_kpts_ for matching) ?
  // SHOULD I HAVE actual_kpts_ or leverage matches_ ?
  Keypoints new_kpts;
  new_kpts.reserve(detected.size());
  for (const auto& p : detected)
  {
    new_kpts.emplace_back(p.x, p.y, 2.0f);
  }

  // TEST
  cv::drawKeypoints(cam.image_, new_kpts, cam.image_, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow("Image with points", cam.image_);
  cv::waitKey();
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

  // Sort detected keypoints based on fast score
  std::sort(cell_kpts.begin(), cell_kpts.end(),
            [](const cv::KeyPoint& pre, const cv::KeyPoint& post) { return pre.response > post.response; });

  // Cap keypoints to max_kpts_per_cell_, keeping the ones with the highest score
  assert((max_kpts_per_cell_.load() - previous_kpts_.size()) > 0);
  cell_kpts.resize(std::min(cell_kpts.size(), (max_kpts_per_cell_.load() - previous_kpts_.size())));
}

}  // namespace msceqf