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

#include "msceqf/state/state.hpp"

#include "utils/logger.hpp"
#include "utils/tools.hpp"

namespace msceqf
{

MSCEqFState::MSCEqFState(const StateOptions& opts) : opts_(opts), cov_(), state_(), clones_()
{
  preallocate();

  Matrix15 Dd_cov = Matrix15::Zero();
  Dd_cov.block(0, 0, 9, 9) = opts_.D_init_cov_;
  Dd_cov.block(9, 9, 6, 6) = opts_.delta_init_cov_;

  // Initialize core state variables (Dd, E, L) and their covariance
  // Note that persistent features are delayed initialized
  initializeStateElement(MSCEqFStateElementName::Dd, Dd_cov);
  if (opts.enable_camera_extrinsics_calibration_)
  {
    initializeStateElement(MSCEqFStateElementName::E, opts_.E_init_cov_);
  }
  if (opts.enable_camera_intrinsics_calibration_)
  {
    initializeStateElement(MSCEqFStateElementName::L, opts_.L_init_cov_);
  }
}

MSCEqFState::MSCEqFState(const MSCEqFState& other) : opts_(other.opts_), cov_(), state_(), clones_()
{
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }
  for (const auto& [key, element] : other.clones_)
  {
    clones_[key] = std::make_unique<MSCEqFSE3State>(*element);
  }
  cov_ = other.cov_;
}

MSCEqFState::MSCEqFState(MSCEqFState&& other) noexcept
    : opts_(std::move(other.opts_))
    , cov_(std::move(other.cov_))
    , state_(std::move(other.state_))
    , clones_(std::move(other.clones_))
{
}

MSCEqFState& MSCEqFState::operator=(const MSCEqFState& other)
{
  state_.clear();
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }
  clones_.clear();
  for (const auto& [key, element] : other.clones_)
  {
    clones_[key] = std::make_unique<MSCEqFSE3State>(*element);
  }
  cov_.resize(other.cov_.rows(), other.cov_.cols());
  cov_ = other.cov_;
  opts_ = other.opts_;
  return *this;
}

MSCEqFState& MSCEqFState::operator=(MSCEqFState&& other) noexcept
{
  opts_ = std::move(other.opts_);
  state_ = std::move(other.state_);
  clones_ = std::move(other.clones_);
  cov_ = std::move(other.cov_);
  opts_ = std::move(other.opts_);
  return *this;
}

MSCEqFState ::~MSCEqFState()
{
  state_.clear();
  clones_.clear();
  cov_.resize(0, 0);
}

const SE23& MSCEqFState::D() const
{
  return std::static_pointer_cast<MSCEqFSDBState>(state_.at(MSCEqFStateElementName::Dd))->Dd_.D();
}

const SE3 MSCEqFState::B() const
{
  return std::static_pointer_cast<MSCEqFSDBState>(state_.at(MSCEqFStateElementName::Dd))->Dd_.B();
}

const SE3 MSCEqFState::C() const
{
  return std::static_pointer_cast<MSCEqFSDBState>(state_.at(MSCEqFStateElementName::Dd))->Dd_.C();
}

const Vector6& MSCEqFState::delta() const
{
  return std::static_pointer_cast<MSCEqFSDBState>(state_.at(MSCEqFStateElementName::Dd))->Dd_.delta();
}

const SE3& MSCEqFState::E() const
{
  return std::static_pointer_cast<MSCEqFSE3State>(state_.at(MSCEqFStateElementName::E))->E_;
}

const In& MSCEqFState::L() const
{
  return std::static_pointer_cast<MSCEqFInState>(state_.at(MSCEqFStateElementName::L))->L_;
}

const SOT3& MSCEqFState::Q(const uint& feat_id) const
{
  return std::static_pointer_cast<MSCEqFSOT3State>(state_.at(feat_id))->Q_;
}

const MatrixX& MSCEqFState::Cov() const { return cov_; }

const MatrixX MSCEqFState::CovBlock(const MSCEqFStateKey& key) const
{
  return cov_.block(getPtr(key)->getIndex(), getPtr(key)->getIndex(), getPtr(key)->getDof(), getPtr(key)->getDof());
}

void MSCEqFState::preallocate()
{
  size_t num_elements = 1 + opts_.num_persistent_features_;
  if (opts_.enable_camera_extrinsics_calibration_)
  {
    ++num_elements;
  }
  if (opts_.enable_camera_intrinsics_calibration_)
  {
    ++num_elements;
  }
  state_.reserve(num_elements);
}

void MSCEqFState::initializeStateElement(const MSCEqFStateKey& key, const MatrixX& cov_block)
{
  assert(key.valueless_by_exception() == false);

  // Get index (actual size of covairance)
  uint idx = cov_.rows();

  bool success = false;
  if (std::holds_alternative<MSCEqFStateElementName>(key))
  {
    switch (std::get<MSCEqFStateElementName>(key))
    {
      case MSCEqFStateElementName::Dd:
        success = insertStateElement(key, std::move(createMSCEqFStateElement<MSCEqFSDBState>(idx)));
        break;
      case MSCEqFStateElementName::E:
        success = insertStateElement(key, std::move(createMSCEqFStateElement<MSCEqFSE3State>(idx)));
        break;
      case MSCEqFStateElementName::L:
        success = insertStateElement(key, std::move(createMSCEqFStateElement<MSCEqFInState>(idx)));
        break;
    }
  }
  else
  {
    success = insertStateElement(std::get<uint>(key), std::move(createMSCEqFStateElement<MSCEqFSOT3State>(idx)));
  }

  if (success)
  {
    uint size_increment = state_[key]->getDof();
    cov_.conservativeResizeLike(MatrixX::Zero(idx + size_increment, idx + size_increment));

    assert(cov_block.rows() == cov_block.cols());
    assert(cov_block.rows() == size_increment);

    cov_.block(idx, idx, size_increment, size_increment) = cov_block;

    // utils::Logger::debug("Assigned covariance block from (" + std::to_string(idx) + "," + std::to_string(idx) +
    //                      "), to (" + std::to_string(idx + size_increment) + "," + std::to_string(idx +
    //                      size_increment) +
    //                      "). Full covariance is now: \n" +
    //                      static_cast<std::ostringstream&>(std::ostringstream() << cov_).str());
  }
  else
  {
    utils::Logger::debug("Failed to initialize new state element with key: " + toString(key));
  }
}

void MSCEqFState::stochasticCloning(const fp& timestamp)
{
  const uint old_size = cov_.rows();

  auto ptr = std::static_pointer_cast<MSCEqFSE3State>(state_.at(MSCEqFStateElementName::E));
  assert(ptr != nullptr);

  MSCEqFSE3StateSharedPtr clone = std::make_shared<MSCEqFSE3State>(*ptr);
  clone->updateIndex(old_size);

  if (clones_.try_emplace(timestamp, clone).second)
  {
    utils::Logger::debug("Created MSCEqF Clone element at time: " + std::to_string(timestamp));

    const uint& E_idx = ptr->getIndex();
    const uint& size_increment = clone->getDof();

    cov_.conservativeResizeLike(MatrixX::Zero(old_size + size_increment, old_size + size_increment));

    cov_.block(old_size, old_size, size_increment, size_increment) =
        cov_.block(E_idx, E_idx, size_increment, size_increment);
    cov_.block(0, old_size, old_size, size_increment) = cov_.block(0, E_idx, old_size, size_increment);
    cov_.block(old_size, 0, size_increment, old_size) = cov_.block(E_idx, 0, size_increment, old_size);
  }
  else
  {
    utils::Logger::debug("Failed to create MSCEqF Clone element at time: " + std::to_string(timestamp));
  }
}

void MSCEqFState::marginalizeCloneAt(const fp& timestamp)
{
  MSCEqFSE3StateSharedPtr clone_to_remove = clones_.at(timestamp);
  const uint& idx = clone_to_remove->getIndex();
  const uint& size = clone_to_remove->getDof();

  // Create a binary mask to slice the covariance
  Eigen::MatrixXd mask = Eigen::MatrixXd::Ones(cov_.rows(), cov_.cols());
  mask.block(idx, 0, size, cov_.cols()) = Eigen::MatrixXd::Zero(size, cov_.cols());
  mask.block(0, idx, cov_.rows(), size) = Eigen::MatrixXd::Zero(cov_.rows(), size);

  // Slice the covariance
  cov_ = (mask.array() == 1).select(cov_, 0);
  cov_.conservativeResize(cov_.rows() - size, cov_.cols() - size);

  // Remove clone
  clones_.erase(timestamp);
}

const fp& MSCEqFState::cloneTimestampToMarginalize() const { return clones_.cbegin()->first; }

bool MSCEqFState::insertStateElement(const MSCEqFStateKey& key, MSCEqFStateElementUniquePtr ptr)
{
  assert(ptr != nullptr);
  if (state_.try_emplace(key, std::move(ptr)).second)
  {
    utils::Logger::info("Created MSCEqF State element [" + toString(key) + "]");
    return true;
  }
  return false;
}

const MSCEqFStateElementSharedPtr& MSCEqFState::getPtr(const MSCEqFStateKey& key) const
{
  assert(key.valueless_by_exception() == false);
  if (std::holds_alternative<MSCEqFStateElementName>(key))
  {
    return state_.at(std::get<MSCEqFStateElementName>(key));
  }
  else
  {
    return state_.at(std::get<uint>(key));
  }
}

const MSCEqFState MSCEqFState::Random() const
{
  MSCEqFState result(*this);

  for (auto& [key, ptr] : result.state_)
  {
    assert(key.valueless_by_exception() == false);

    if (std::holds_alternative<MSCEqFStateElementName>(key))
    {
      switch (std::get<MSCEqFStateElementName>(key))
      {
        case MSCEqFStateElementName::Dd:
          std::static_pointer_cast<MSCEqFSDBState>(ptr)->Dd_ =
              SDB(SE23(Quaternion::UnitRandom(), {Vector3::Random(), Vector3::Random()}), Vector6::Random());
          break;
        case MSCEqFStateElementName::E:
          std::static_pointer_cast<MSCEqFSE3State>(ptr)->E_ = SE3(Quaternion::UnitRandom(), {Vector3::Random()});
          break;
        case MSCEqFStateElementName::L:
          Vector4 intr = 100 * Vector4::Random().cwiseAbs();
          std::static_pointer_cast<MSCEqFInState>(ptr)->L_ = In(intr.x(), intr.y(), intr.z(), intr.w());
          break;
      }
    }
    else
    {
      std::static_pointer_cast<MSCEqFSOT3State>(ptr)->Q_ = SOT3(Quaternion::UnitRandom(), utils::random<fp>(0, 10));
    }
  }
  return result;
}

const MSCEqFState MSCEqFState::operator*(const MSCEqFState& other) const
{
  MSCEqFState result(*this);

  for (auto& [key, ptr] : other.state_)
  {
    assert(key.valueless_by_exception() == false);

    if (std::holds_alternative<MSCEqFStateElementName>(key))
    {
      switch (std::get<MSCEqFStateElementName>(key))
      {
        case MSCEqFStateElementName::Dd:
          std::static_pointer_cast<MSCEqFSDBState>(result.state_.at(key))
              ->Dd_.multiplyRight(std::static_pointer_cast<MSCEqFSDBState>(ptr)->Dd_);
          break;
        case MSCEqFStateElementName::E:
          std::static_pointer_cast<MSCEqFSE3State>(result.state_.at(key))
              ->E_.multiplyRight(std::static_pointer_cast<MSCEqFSE3State>(ptr)->E_);
          break;
        case MSCEqFStateElementName::L:
          std::static_pointer_cast<MSCEqFInState>(result.state_.at(key))
              ->L_.multiplyRight(std::static_pointer_cast<MSCEqFInState>(ptr)->L_);
          break;
      }
    }
    else
    {
      std::static_pointer_cast<MSCEqFSOT3State>(result.state_.at(key))
          ->Q_.multiplyRight(std::static_pointer_cast<MSCEqFSOT3State>(ptr)->Q_);
    }
  }
  return result;
}

std::string MSCEqFState::toString(const MSCEqFStateKey& key)
{
  std::string name;
  if (std::holds_alternative<MSCEqFStateElementName>(key))
  {
    switch (std::get<MSCEqFStateElementName>(key))
    {
      case MSCEqFStateElementName::Dd:
        name = "Semi Direct Bias (D, delta)";
        break;
      case MSCEqFStateElementName::E:
        name = "Special Euclidean (E)";
        break;
      case MSCEqFStateElementName::L:
        name = "Intrinsic (L)";
        break;
    }
  }
  else
  {
    name = "Scaled Orthogonal Transforms (SOT3) associated with feature id: " + std::to_string(std::get<uint>(key));
  }
  return name;
}

}  // namespace msceqf