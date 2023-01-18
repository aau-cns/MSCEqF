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

MSCEqFState::MSCEqFState(const StateOptions& opts) : cov_(), state_(), clones_(), opts_(opts)
{
  // Preallocate state memory based on given options
  preallocate();

  // Define Semi Direct Bias initial covariance
  Matrix15 Dd_cov = Matrix15::Zero();
  Dd_cov.block(0, 0, 9, 9) = opts_.D_init_cov_;
  Dd_cov.block(9, 9, 6, 6) = opts_.delta_init_cov_;

  // Initialize core state variables (Dd, E, L) and their covariance
  // Persistent features are delayed initialized
  initializeStateElement(MSCEqFStateElementName::Dd, Dd_cov);
  if (opts.enable_camera_extrinsic_calibration_)
  {
    initializeStateElement(MSCEqFStateElementName::E, opts_.E_init_cov_);
  }
  if (opts.enable_camera_intrinsic_calibration_)
  {
    initializeStateElement(MSCEqFStateElementName::L, opts_.L_init_cov_);
  }
}

MSCEqFState::MSCEqFState(const MSCEqFState& other) : cov_(), state_(), clones_(), opts_(other.opts_)
{
  // Copy state_
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }

  // Copy clones_
  for (const auto& [key, element] : other.clones_)
  {
    clones_[key] = std::make_unique<MSCEqFSE3State>(*element);
  }

  // Copy covariance
  cov_ = other.cov_;
}

MSCEqFState::MSCEqFState(MSCEqFState&& other) noexcept
    : cov_(std::move(other.cov_))
    , state_(std::move(other.state_))
    , clones_(std::move(other.clones_))
    , opts_(std::move(other.opts_))
{
}

MSCEqFState& MSCEqFState::operator=(const MSCEqFState& other)
{
  // Copy state_
  state_.clear();
  for (const auto& [key, element] : other.state_)
  {
    state_[key] = element->clone();
  }

  // Copy clones_
  clones_.clear();
  for (const auto& [key, element] : other.clones_)
  {
    clones_[key] = std::make_unique<MSCEqFSE3State>(*element);
  }

  // Copy covariance
  cov_.resize(other.cov_.rows(), other.cov_.cols());
  cov_ = other.cov_;

  return *this;
}

MSCEqFState& MSCEqFState::operator=(MSCEqFState&& other) noexcept
{
  opts_ = std::move(other.opts_);
  state_ = std::move(other.state_);
  clones_ = std::move(other.clones_);
  cov_ = std::move(other.cov_);
  return *this;
}

MSCEqFState ::~MSCEqFState()
{
  state_.clear();
  clones_.clear();
  cov_.resize(0, 0);
}

void MSCEqFState::preallocate()
{
  // Preallocate space on state map
  size_t num_elements = 1 + opts_.num_persistent_features_;
  if (opts_.enable_camera_extrinsic_calibration_)
  {
    ++num_elements;
  }
  if (opts_.enable_camera_intrinsic_calibration_)
  {
    ++num_elements;
  }
  state_.reserve(num_elements);

  // Preallocate space on clones map
  clones_.reserve(opts_.num_clones_);
}

void MSCEqFState::initializeStateElement(const MSCEqFStateKey& key, const MatrixX& cov_block)
{
  assert(key.valueless_by_exception() == false);

  // Get index (actual size of covairance)
  uint idx = cov_.rows();

  // Create the MSCEqF state element checking if the key holds a element name or a feaure id
  if (std::holds_alternative<MSCEqFStateElementName>(key))
  {
    switch (std::get<MSCEqFStateElementName>(key))
    {
      case MSCEqFStateElementName::Dd:
        insertStateElement(key, std::move(createMSCEqFStateElement<MSCEqFSDBState>(idx)));
        break;
      case MSCEqFStateElementName::E:
        insertStateElement(key, std::move(createMSCEqFStateElement<MSCEqFSE3State>(idx)));
        break;
      case MSCEqFStateElementName::L:
        insertStateElement(key, std::move(createMSCEqFStateElement<MSCEqFInState>(idx)));
        break;
    }
  }
  else
  {
    insertStateElement(std::get<uint>(key), std::move(createMSCEqFStateElement<MSCEqFSOT3State>(idx)));
  }

  // Resize covariance
  uint size_increment = state_[key]->getDof();
  cov_.conservativeResize(idx + size_increment, idx + size_increment);

  assert(cov_block.rows() == cov_block.cols());
  assert(cov_block.rows() == size_increment);

  // Assign new covariance block
  cov_.block(idx, idx, size_increment, size_increment) = cov_block;

  // Log
  utils::Logger::debug("Assigned covariance block from (" + std::to_string(idx) + "," + std::to_string(idx) +
                       "), to (" + std::to_string(idx + size_increment) + "," + std::to_string(idx + size_increment) +
                       ")\n" + static_cast<std::ostringstream&>(std::ostringstream() << cov_block).str());
}

void MSCEqFState::insertStateElement(const MSCEqFStateKey& key, MSCEqFStateElementUniquePtr ptr)
{
  assert(ptr != nullptr);
  state_.try_emplace(key, std::move(ptr));

  // Log
  utils::Logger::info("Created MSCEqF State element [" + toString(key) + "]");
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

const MSCEqFState MSCEqFState::Random() const
{
  // Copy this
  MSCEqFState result(*this);

  // Assign random values to state map
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
  // Copy this
  MSCEqFState result(*this);

  // Assign composed (multiplied) values to state map
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