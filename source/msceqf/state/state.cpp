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
  if (opts_.enable_camera_extrinsics_calibration_)
  {
    initializeStateElement(MSCEqFStateElementName::E, opts_.E_init_cov_);
  }
  if (opts_.enable_camera_intrinsics_calibration_)
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
    clones_[key] = element->clone();
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
    clones_[key] = element->clone();
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

const SE3& MSCEqFState::clone(const fp& timestamp) const
{
  return std::static_pointer_cast<MSCEqFSE3State>(clones_.at(timestamp))->E_;
}

const uint& MSCEqFState::index(const MSCEqFKey& key) const { return getPtr(key)->getIndex(); }

const uint& MSCEqFState::dof(const MSCEqFKey& key) const { return getPtr(key)->getDof(); }

const MatrixX& MSCEqFState::cov() const { return cov_; }

const MatrixX MSCEqFState::covBlock(const MSCEqFKey& key) const
{
  return cov_.block(getPtr(key)->getIndex(), getPtr(key)->getIndex(), getPtr(key)->getDof(), getPtr(key)->getDof());
}

const MatrixX MSCEqFState::subCov(const std::vector<MSCEqFKey>& keys) const
{
  assert(!keys.empty());

  // All the blocks of the covariance submatrix in column-major order.
  std::vector<MatrixX> column_major_blocks;

  uint total_size = 0;
  for (size_t c = 0; c < keys.size(); ++c)
  {
    const uint& col_idx = getPtr(keys[c])->getIndex();
    const uint& col_dof = getPtr(keys[c])->getDof();

    total_size += col_dof;

    for (size_t r = 0; r < keys.size(); ++r)
    {
      const uint& row_idx = r == c ? col_idx : getPtr(keys[r])->getIndex();
      const uint& row_dof = r == c ? col_dof : getPtr(keys[r])->getDof();

      column_major_blocks.emplace_back(cov_.block(row_idx, col_idx, row_dof, col_dof));
    }
  }

  MatrixX sub_cov = MatrixX::Zero(total_size, total_size);

  // Assign blocks in column_major_blocks to sub_cov.
  uint cur_row = 0;
  uint cur_col = 0;
  for (uint i = 0; i < column_major_blocks.size(); ++i)
  {
    sub_cov.block(cur_row, cur_col, column_major_blocks[i].rows(), column_major_blocks[i].cols()) =
        column_major_blocks[i];

    cur_row += column_major_blocks[i].rows();

    if (cur_row == total_size)
    {
      cur_row = 0;
      cur_col += column_major_blocks[i].cols();
    }
  }

  return sub_cov;
}

const MatrixX MSCEqFState::subCovCols(const std::vector<MSCEqFKey>& keys) const
{
  assert(!keys.empty());

  uint columns_size = 0;
  for (const auto& key : keys)
  {
    columns_size += getPtr(key)->getDof();
  }

  const auto& rows = cov_.rows();
  MatrixX sub_cov = MatrixX::Zero(rows, columns_size);

  uint cur_col = 0;
  for (size_t c = 0; c < keys.size(); ++c)
  {
    const auto& size = getPtr(keys[c])->getDof();
    sub_cov.middleCols(cur_col, size) = cov_.middleCols(getPtr(keys[c])->getIndex(), size);
    cur_col += size;
  }

  return sub_cov;
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

  MSCEqFStateElementSharedPtr ptr = nullptr;
  if (opts_.enable_camera_extrinsics_calibration_)
  {
    ptr = state_.at(MSCEqFStateElementName::E);
  }
  assert(ptr != nullptr);

  MSCEqFStateElementUniquePtr clone = ptr->clone();
  clone->updateIndex(old_size);

  if (insertCloneElement(timestamp, std::move(clone)))
  {
    utils::Logger::debug("Created MSCEqF Clone element at time: " + std::to_string(timestamp));

    const uint& E_idx = ptr->getIndex();
    const uint& size_increment = ptr->getDof();

    cov_.conservativeResize(old_size + size_increment, old_size + size_increment);

    cov_.block(old_size, old_size, size_increment, size_increment) =
        cov_.block(E_idx, E_idx, size_increment, size_increment).eval();
    cov_.block(0, old_size, old_size, size_increment) = cov_.block(0, E_idx, old_size, size_increment).eval();
    cov_.block(old_size, 0, size_increment, old_size) = cov_.block(E_idx, 0, size_increment, old_size).eval();

    // cov_ = 0.5 * (cov_ + cov_.transpose()).eval();

    assert((cov_.middleCols(E_idx, 6) - cov_.middleCols(old_size, 6)).norm() < 1e-12);
    assert((cov_.middleRows(E_idx, 6) - cov_.middleRows(old_size, 6)).norm() < 1e-12);
    assert((cov_.block(E_idx, E_idx, 6, 6) - cov_.block(old_size, old_size, 6, 6)).norm() < 1e-12);
    assert((cov_ - cov_.transpose()).norm() < 1e-12);
  }
  else
  {
    utils::Logger::debug("Failed to create MSCEqF Clone element at time: " + std::to_string(timestamp));
  }
}

void MSCEqFState::marginalizeCloneAt(const fp& timestamp)
{
  const auto& clone_to_remove = clones_.at(timestamp);
  const uint& idx = clone_to_remove->getIndex();
  const uint& size = clone_to_remove->getDof();

  const Eigen::Index rows = cov_.rows();
  const Eigen::Index cols = cov_.cols();

  cov_.block(idx, 0, rows - idx - size, cols) = cov_.block(idx + size, 0, rows - idx - size, cols).eval();
  cov_.block(0, idx, rows, cols - idx - size) = cov_.block(0, idx + size, rows, cols - idx - size).eval();
  cov_.conservativeResize(rows - size, cols - size);

  for (auto& [timestamp, clone] : clones_)
  {
    clone->updateIndex(clone->getIndex() - size);
  }

  clones_.erase(timestamp);

  utils::Logger::debug("Marginalized MSCEqF Clone element at time: " + std::to_string(timestamp));
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

bool MSCEqFState::insertCloneElement(const fp& timestamp, MSCEqFStateElementUniquePtr ptr)
{
  assert(ptr != nullptr);
  if (clones_.try_emplace(timestamp, std::move(ptr)).second)
  {
    utils::Logger::info("Created MSCEqF Clone element at time: " + std::to_string(timestamp));
    return true;
  }
  return false;
}

const MSCEqFStateElementSharedPtr& MSCEqFState::getPtr(const MSCEqFKey& key) const
{
  assert(key.valueless_by_exception() == false);
  if (std::holds_alternative<MSCEqFStateKey>(key))
  {
    const auto& state_key = std::get<MSCEqFStateKey>(key);
    if (std::holds_alternative<MSCEqFStateElementName>(state_key))
    {
      return state_.at(std::get<MSCEqFStateElementName>(state_key));
    }
    else
    {
      return state_.at(std::get<uint>(state_key));
    }
  }
  else
  {
    return clones_.at(std::get<fp>(key));
  }
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

}  // namespace msceqf