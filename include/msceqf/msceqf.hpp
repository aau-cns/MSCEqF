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

#include "msceqf/filter/propagator.hpp"
#include "msceqf/options/msceqf_option_parser.hpp"
#include "msceqf/state/state.hpp"
#include "msceqf/system/system.hpp"

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
   * @brief This function provide a simple interface for processing measurements of different kind.
   *
   * @tparam T Type of the measurement
   * @param meas measurement
   */
  template <typename T>
  void processMeasurement(const T& meas)
  {
    if constexpr (std::is_same_v<T, Imu>)
    {
      processImuMeasurement(meas);
    }
    else
    {
      processCameraMeasurement(meas);
    }
  }

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
  void processCameraMeasurement(const Camera& cam);

  OptionParser parser_;  //!< The parser to parse all the configuration from a yaml file
  MSCEqFOptions opts_;   //!< All the MSCEqF options

  MSCEqFState X_;    //!< The state of the MSCEqF
  SystemState xi0_;  //!< The origin state of the System

  Propagator propagator_;  //!< The MSCEqF propagator

  fp timestamp_ = -1;  //!< The timestamp of the actual estimate

  bool is_filter_initialized_ = false;  //!< Flag that indicates that the filter is initialized
};

}  // namespace msceqf

#endif  // MSCEQF_HPP