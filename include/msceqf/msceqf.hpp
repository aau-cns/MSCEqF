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

#ifndef MSCEQF_HPP
#define MSCEQF_HPP

#include <future>

#include "msceqf/filter/initializer/static_initializer.hpp"
#include "msceqf/filter/propagator/propagator.hpp"
#include "msceqf/filter/updater/updater.hpp"
#include "msceqf/options/msceqf_option_parser.hpp"
#include "msceqf/state/state.hpp"
#include "msceqf/system/system.hpp"
#include "vision/track_manager.hpp"
#include "utils/visualizer.hpp"

namespace msceqf
{

class MSCEqF
{
 public:
  /**
   * @brief MSCEqF Constructor
   *
   * @param params_filepath filepath of the parameter file to be parsed
   */
  MSCEqF(const std::string& params_filepath);

  /**
   * @brief MSCEqF Constructor with given initial orientation
   *
   * @param params_filepath filepath of the parameter file to be parsed
   * @param q initial otientation
   */
  MSCEqF(const std::string& params_filepath, const Quaternion& q);

  /**
   * @brief This function provide a simple interface for processing measurements of different kind.
   *
   * @tparam T Type of the measurement
   * @param meas measurement
   */
  void processMeasurement(const Imu& meas) { processImuMeasurement(meas); }
  void processMeasurement(Camera& meas) { processCameraMeasurement(meas); }
  void processMeasurement(const TriangulatedFeatures& meas) { processFeaturesMeasurement(meas); }

  /**
   * @brief Get a constant reference to the MSCEqF options
   *
   * @return const MSCEqFOptions&
   */
  const MSCEqFOptions& options() const;

  /**
   * @brief Get a constant reference to the MSCEqF state options
   *
   * @return const StateOptions&
   */
  const StateOptions& stateOptions() const;

  /**
   * @brief Get a constant reference to the covariance matrix of the MSCEqF state
   *
   * @return const MatrixX&
   */
  const MatrixX& Covariance() const;

  /**
   * @brief Get a constant copy of the estimated state
   *
   * @return const SystemState
   */
  const SystemState stateEstimate() const;

 private:
  /**
   * @brief Process a single IMU measurement. This method will fill the internal IMU measurement buffer, that will
   * be used for propagation upon receiveing a camera measurement.
   *
   * @param imu
   */
  void processImuMeasurement(const Imu& imu);

  /**
   * @brief Process a single Camera measurement.
   *
   * @param cam
   */
  void processCameraMeasurement(Camera& cam);

  /**
   * @brief Process triangulated features measurement.
   *
   * @param features
   */
  void processFeaturesMeasurement(const TriangulatedFeatures& features);

  OptionParser parser_;  //!< The parser to parse all the configuration from a yaml file
  MSCEqFOptions opts_;   //!< All the MSCEqF options

  MSCEqFState X_;          //!< The state of the MSCEqF
  const SystemState xi0_;  //!< The origin state of the System

  TrackManager track_manager_;     //!< The MSCEqF track manager
  StaticInitializer initializer_;  //!< The MSCEqF static initializer
  Propagator propagator_;          //!< The MSCEqF propagator
  Updater updater_;                //!< The MSCEqF updater
  Visualizer visualizer_;          //<! The MSCEqF visualizer

  std::unordered_set<uint> ids_to_update_;  //!< Ids of track to update

  fp timestamp_;  //!< The timestamp of the actual estimate

  bool is_filter_initialized_;  //!< Flag that indicates that the filter is initialized
};

}  // namespace msceqf

#endif  // MSCEQF_HPP