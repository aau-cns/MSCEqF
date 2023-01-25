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

#include "msceqf/msceqf.hpp"

namespace msceqf
{

MSCEqF::MSCEqF(const std::string& params_filepath)
    : parser_(params_filepath)
    , opts_(parser_.parseOptions())
    , X_(opts_.state_options_)
    , xi0_(opts_.state_options_)
    , initializer_(opts_.init_options_)
    , propagator_(opts_)
{
}

void MSCEqF::processImuMeasurement(const Imu& imu)
{
  if (!is_filter_initialized_)
  {
    utils::Logger::info("Collecting IMU measurements for static initialization.");
    initializer_.insertImu(imu);
    return;
  }

  if (imu.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received IMU measurement older than actual state estimate. Discarding measurement.");
  }
  else
  {
    propagator_.insertImu(X_, xi0_, imu, timestamp_);
  }
}

void MSCEqF::processCameraMeasurement(const Camera& cam)
{
  if (!is_filter_initialized_)
  {
    initializer_.detectMotion(/* camera meas */);
  }

  if (cam.timestamp_ < timestamp_)
  {
    utils::Logger::warn("Received Camera measurement older than actual state estimate. Discarding measurement.");
    return;
  }

  // [TODO] manage stochastic cloning
  if (!propagator_.propagate(X_, xi0_, timestamp_, cam.timestamp_))
  {
    utils::Logger::err("Propagation failure.");
  }
}

const MSCEqFOptions& MSCEqF::options() const { return opts_; }

const StateOptions& MSCEqF::stateOptions() const { return opts_.state_options_; }

const MatrixX& MSCEqF::Covariance() const { return X_.Cov(); }

const SystemState MSCEqF::stateEstimate() const { return Symmetry::phi(X_, xi0_); }

}  // namespace msceqf