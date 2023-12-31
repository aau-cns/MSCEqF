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
#include "msceqf/filter/updater/zero_velocity_updater.hpp"
#include "msceqf/options/msceqf_option_parser.hpp"
#include "msceqf/state/state.hpp"
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
   * @brief Process IMU measurement.
   *
   * @param meas IMU measurement
   */
  void processMeasurement(const Imu& meas) { processImuMeasurement(meas); }

  /**
   * @brief Process camera measurement.
   *
   * @param meas Camera measurement
   */
  void processMeasurement(Camera& meas) { processCameraMeasurement(meas); }

  /**
   * @brief Process triangulated features measurement.
   *
   * @param meas Triangulated features measurement
   */
  void processMeasurement(TriangulatedFeatures& meas) { processFeaturesMeasurement(meas); }

  /**
   * @brief Get a constant reference to the MSCEqF options
   *
   * @return Options
   */
  const MSCEqFOptions& options() const;

  /**
   * @brief Get a constant reference to the MSCEqF state options
   *
   * @return State options
   */
  const StateOptions& stateOptions() const;

  /**
   * @brief Get a constant reference to the covariance matrix of the MSCEqF state
   *
   * @return Covariance matrix
   */
  const MatrixX& covariance() const;

  /**
   * @brief Get the covariance of the navigation states (D, delta)
   *
   * @return Covariance matrix of the navigation states
   */
  const MatrixX coreCovariance() const;

  /**
   * @brief Get a constant reference to the estimated state
   *
   * @return System state (homogeneous space element) reperesenting the state estimate
   */
  const SystemState& stateEstimate() const;

  /**
   * @brief Get a constant reference to the origin state
   *
   * @return System state (homogeneous space element) reperesenting the state origin
   */
  const SystemState& stateOrigin() const;

  /**
   * @brief Set origin xi0 with given state programatically
   *
   * @param T0 Origin extended pose
   * @param b0 Origin bias
   * @param timestamp initial timestamp
   */
  void setGivenOrigin(const SE23& T0, const Vector6& b0, const fp& timestamp);

  /**
   * @brief Get the processed image with overlayed tracks
   *
   * @return OpenCV matrix representing the image with overlayed tracks
   */
  const cv::Mat3b imageWithTracks(const Camera& cam) const;

  /**
   * @brief Visualize the processed image with overlayed tracks
   *
   * @param cam Camera measurement
   */
  void visualizeImageWithTracks(const Camera& cam) const;

  /**
   * @brief Check if the filter is initialized
   *
   * @return true if the filter is initialized, false otherwise
   */
  [[nodiscard]] const bool& isInit() const;

  /**
   * @brief Check if a zero velocity update has been performed
   *
   * @return true if the a zero velocity update has been performed, false otherwise
   */
  [[nodiscard]] const bool& zvuPerformed() const;

 private:
  /**
   * @brief Process a single IMU measurement. This method will fill the internal IMU measurement buffer, that will
   * be used for propagation upon receiveing a camera measurement.
   *
   * @param imu IMU measurement
   */
  void processImuMeasurement(const Imu& imu);

  /**
   * @brief Process a single Camera measurement. This method first perform propagation of the filter state from the
   * previous timestamp to the actual timestamp using the IMU measurement collected in between camera images. After
   * propagation stochastic cloning is performed. Then ids of the feature that either went out of the field-of-view or
   * that are active once the window of clone has been filled are collected and used to perform a filter update.
   * Finally, tracks associated with features used in the update are removed and past clones are marginalized.
   *
   * @note Propagation of the filter, stochastic cloning and image processing are parallelized.
   *
   * @param cam Camera measurement
   */
  void processCameraMeasurement(Camera& cam);

  /**
   * @brief Process triangulated features measurement. This method first perform propagation of the filter state from
   * the previous timestamp to the actual timestamp using the IMU measurement collected in between camera images. After
   * propagation stochastic cloning is performed. Then ids of the feature that either went out of the field-of-view or
   * that are active once the window of clone has been filled are collected and used to perform a filter update.
   * Finally, tracks associated with features used in the update are removed and past clones are marginalized.
   *
   * @note Propagation of the filter, stochastic cloning and image processing are parallelized.
   *
   * @param features Triangulated features measurement
   */
  void processFeaturesMeasurement(TriangulatedFeatures& features);

  /**
   * @brief Try to initialize the origin at the time of the given features measurement.
   * This method either perform static initialization waiting for motion to be detected or dircetly initialize origin
   * and filter if zero velocity update is enabled.
   *
   * @param cam Camera measurement
   */
  void initialize(Camera& cam);

  /**
   * @brief Try to initialize the origin at the time of the given camera measurement.
   * This method either perform static initialization waiting for motion to be detected or dircetly initialize origin
   * and filter if zero velocity update is enabled.
   *
   * @param features features measurement
   */
  void initialize(TriangulatedFeatures& features);

  /**
   * @brief Set origin xi0 with given state from parameters file
   *
   */
  void setGivenOrigin();

  /**
   * @brief Log initial condition of the filter
   *
   */
  void logInit() const;

  OptionParser parser_;  //!< The parser to parse all the configuration from a yaml file
  MSCEqFOptions opts_;   //!< All the MSCEqF options

  SystemState xi0_;  //!< The origin state of the System
  MSCEqFState X_;    //!< The state of the MSCEqF
  SystemState xi_;   //!< The state of the System in the homogeneous space

  TrackManager track_manager_;     //!< The MSCEqF track manager
  Checker checker_;                //!< The MSCEqF checker
  StaticInitializer initializer_;  //!< The MSCEqF static initializer
  Propagator propagator_;          //!< The MSCEqF propagator
  Updater updater_;                //!< The MSCEqF updater
  ZeroVelocityUpdater zvupdater_;  //!< The MSCEqF zero velocity updater
  Visualizer visualizer_;          //<! The MSCEqF visualizer

  std::unordered_set<uint> ids_to_update_;  //!< Ids of track to update

  fp timestamp_;  //!< The timestamp of the actual estimate

  bool is_filter_initialized_;  //!< Flag that indicates that the filter is initialized
  bool zvu_performed_;          //!< Flag that indicates that the zero velocity update has been performed
};

}  // namespace msceqf

#endif  // MSCEQF_HPP