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

#ifndef PROPAGATOR_HPP
#define PROPAGATOR_HPP

#include "msceqf/options/msceqf_options.hpp"
#include "msceqf/symmetry/symmetry.hpp"
#include "msceqf/system/sensor_data.hpp"
#include "msceqf/system/system.hpp"
#include "types/fptypes.hpp"
#include "utils/tools.hpp"

namespace msceqf
{

class Propagator
{
 public:
  using ImuBuffer = std::deque<Imu>;  //!< The Imu measurement buffer

  /**
   * @brief Construct a Propagator object given the options
   *
   * @param opts
   */
  Propagator(const MSCEqFOptions& opts);

  /**
   * @brief insert a new IMU measurement into the imu buffer, if the given IMU measurement has a grater timestamp than
   * the last measurement in the buffer.
   *
   * @param X Actual state estimate
   * @param xi0 origin
   * @param imu
   *
   * @note This function triggers a propagation if the imu buffer reaches its max size.
   * In such case all the entries of the imu buffer except the last one are deleted to avoid unbounded memory growth.
   */
  void insertImu(MSCEqFState& X, const SystemState& xi0, const Imu& imu);

  /**
   * @brief This function implements the mean and covariance propagation from t0 to t1 for the MSCEqF.
   *
   * @param X Actual state estimate
   * @param xi0 origin
   * @param t0 integration period start time (actual state timestamp)
   * @param t1 integration period end time (new state timestamp)
   * @return true if propagation succeeded, false otherwise
   */
  bool propagate(MSCEqFState& X, const SystemState& xi0, const fp& t0, const fp& t1);

 private:
  /**
   * @brief Get IMU readings between t0 and t1 to propagate with, and remove such readings from the IMU buffer.
   * This method will also perform linear interpolation at t1 time if no IMU readings exist with timestamp equal t1 and
   * at least one IMU reading exist with timestamp grater than t1.
   *
   * @param t0 start time
   * @param t1 end time
   * @return ImuBuffer
   *
   * @note This function reqires that the IMU biffer is sorted. This should be enforced by the checks in insertion.
   */
  ImuBuffer getImuReadings(const fp& t0, const fp& t1);

  /**
   * @brief Perform linear interpolation. (1 - alpha) * pre + alpha * post
   *
   * @param pre pre imu measurment
   * @param post post imu measurement
   * @param alpha interpolation coefficient [0, 1]
   * @return Imu
   */
  Imu lerp(const Imu& pre, const Imu& post, const fp& alpha);

  /**
   * @brief Mean propagation for the MSCEqF
   *
   * @param X Actual state estimate
   * @param xi0 origin
   * @param u imu measurement to propagate with
   * @param dt delta time between input measurements
   */
  void propagateMean(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt);

  /**
   * @brief Covariance propagation for the MSCEqF
   *
   * @param X Actual state estimate
   * @param xi0 origin
   * @param u imu measurement to propagate the covariance with (used to compute state matrix)
   * @param dt delta time between input measurements
   */
  void propagateCovariance(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt);

  /**
   * @brief This function computes the core state transition matrix.
   * The core state transition matrix is the state transition matrix for the Dd element of the MSCEqF state, and for the
   * E element if extrinsic calibration is enabled.
   * Based on the given state_transition_order_, this function returns eithe a first order truncation, a second order
   * truncation of matrix exponentaial or the numerical computation of the matrix exponential.
   *
   * @param X Actual state estimate
   * @param xi0 origin
   * @param u imu measurement to propagate the covariance with (used to compute state matrix)
   * @param dt delta time between input measurements
   * @return MatrixX The core state transition matrix Phi
   */
  MatrixX coreStateTransitionMatrix(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt);

  /**
   * @brief This function computes the second order terms of the core state transition matrix.
   *
   * @param A Continuous time state matrix A
   * @param dt delta time between input measurements
   * @param enable_camera_extrinsics_calibration flag
   * @return MatrixX The second order terms of the core state transition matrix Phi
   */
  MatrixX coreSecondOrderPhi(const MatrixX& A, const fp& dt, const bool& enable_camera_extrinsics_calibration);

  /**
   * @brief This function computes the discrete time process noise covariance matrix as B * Q * B^T * dt
   *
   * @param X Actual state estimate
   * @param xi0 origin
   * @param dt delta time between input measurements
   * @return MatrixX Discrete time process noise covariance matrix
   */
  MatrixX inputMatrix(MSCEqFState& X, const SystemState& xi0, const fp& dt);

  ImuBuffer imu_buffer_;  //!< The imu measurement buffer
  Matrix12 Q_;            //!< The continuous time process noise covariance

  int state_transition_order_;  //!< Truncation order of the state transition matrix
  uint imu_buffer_max_size_;    //!< Maximum imu buffer size

  static constexpr fp eps_ = 1e-6;  //!< epsilon for checks
};

}  // namespace msceqf

#endif  // PROPAGATOR_HPP