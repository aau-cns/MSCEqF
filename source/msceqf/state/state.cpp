// Copyright (C) 2023 Alessandro Fornasier, Pieter van Goor.
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

namespace msceqf
{
MSCEqFState::MSCEqFState(const StateOptions& opts) : cov_(), state_(), clones_(), opts_(opts)
{
  // Preallocate state memory based on given options
  size_t num_elements = 1 + opts.num_persistent_features_;
  if (opts_.enable_camera_extrinsic_calibration_)
  {
    ++num_elements;
  }
  if (opts_.enable_camera_intrinsic_calibration_)
  {
    ++num_elements;
  }
  state_.reserve(num_elements);

  // Preallocate clones memory based on given options
  clones_.reserve(opts_.num_clones_);

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
      default:
        // [TODO] needed?
        throw std::invalid_argument("Impossible to create MSCEqF state element, invalid MSCEqF state name provided");
    }
  }
  else
  {
    insertStateElement(std::get<uint>(key), std::move(createMSCEqFStateElement<MSCEqFSOT3State>(idx)));
  }

  // Resize covariance
  uint size_increment = state_.at(key)->getDof();
  cov_.conservativeResize(idx + size_increment, idx + size_increment);

  assert(cov_block.rows() == cov_block.cols());
  assert(cov_block.rows() == size_increment);

  // Assign new covariance block
  cov_.block(idx, idx, size_increment, size_increment) = cov_block;
}

void MSCEqFState::insertStateElement(const MSCEqFStateKey& key, MSCEqFStateElementUniquePtr ptr)
{
  assert(ptr != nullptr);
  state_.try_emplace(key, std::move(ptr));
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

const SE23& MSCEqFState::D() const { return getCastedPtr<MSCEqFSDBState>(MSCEqFStateElementName::Dd)->getDd().D(); }

const SE3 MSCEqFState::B() const { return getCastedPtr<MSCEqFSDBState>(MSCEqFStateElementName::Dd)->getDd().B(); }

const SE3 MSCEqFState::C() const { return getCastedPtr<MSCEqFSDBState>(MSCEqFStateElementName::Dd)->getDd().C(); }

const Vector6& MSCEqFState::delta() const
{
  return getCastedPtr<MSCEqFSDBState>(MSCEqFStateElementName::Dd)->getDd().delta();
}

const SE3& MSCEqFState::E() const { return getCastedPtr<MSCEqFSE3State>(MSCEqFStateElementName::E)->getE(); }

const In& MSCEqFState::L() const { return getCastedPtr<MSCEqFInState>(MSCEqFStateElementName::L)->getL(); }

const SOT3& MSCEqFState::Q(const uint& feat_id) const { return getCastedPtr<MSCEqFSOT3State>(feat_id)->getQ(); }

const MatrixX& MSCEqFState::Cov() const { return cov_; }

const MatrixX MSCEqFState::CovBlock(const MSCEqFStateKey& key)
{
  return cov_.block(getPtr(key)->getIndex(), getPtr(key)->getIndex(), getPtr(key)->getDof(), getPtr(key)->getDof());
}

}  // namespace msceqf