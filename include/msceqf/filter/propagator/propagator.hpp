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
#include "sensors/sensor_data.hpp"
#include "msceqf/state/state.hpp"
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
  Propagator(const PropagatorOptions& opts);

  /**
   * @brief insert a new IMU measurement into the imu buffer, if the given IMU measurement has a grater timestamp than
   * the last measurement in the buffer.
   *
   * @param X Actual state estimate
   * @param xi0 Origin
   * @param imu IMU measurement
   * @param timestamp The actual timestamp. If a propagation is triggered than the timestamp is updated
   *
   * @note This function triggers a propagation if the imu buffer reaches its max size.
   * In such case all the entries of the imu buffer except the last one are deleted to avoid unbounded memory growth.
   */
  void insertImu(MSCEqFState& X, const SystemState& xi0, const Imu& imu, fp& timestamp);

  /**
   * @brief This function implements the mean and covariance propagation from timestamp to new_timestamp for the MSCEqF.
   *
   * @param X Actual state estimate
   * @param xi0 Origin
   * @param timestamp Integration period start time (actual state timestamp). *This will be modified by the propagation*
   * @param new_timestamp Integration period end time (new state timestamp)
   * @return true if propagation succeeded, false otherwise
   */
  bool propagate(MSCEqFState& X, const SystemState& xi0, fp& timestamp, const fp& new_timestamp);

 private:
  /**
   * @brief Get IMU readings between t0 and t1 to propagate with, and remove such readings from the IMU buffer.
   * This method will also perform linear interpolation at t0 time if no IMU readings exist with timestamp equal t0 and
   * at least one IMU reading exist with timestamp smaller than t0.
   * This method will also perform linear interpolation at t1 time if no IMU readings exist with timestamp equal t1 and
   * at least one IMU reading exist with timestamp grater than t1.
   *
   * @param t0 Start time
   * @param t1 End time
   * @return IMU buffer
   *
   * @note This method reqires that the IMU buffer is sorted. This should be enforced by the checks in insertion.
   * @note This method has been written with specific care to avoid iterator invalidation.
   */
  ImuBuffer getImuReadings(const fp& t0, const fp& t1);

  /**
   * @brief Perform linear interpolation. (1 - alpha) * pre + alpha * post
   *
   * @param pre Pre imu measurment (at t0)
   * @param post Post imu measurement (at t1)
   * @param alpha Interpolation coefficient [0, 1]
   * @return Interpolated IMU measurement
   */
  Imu lerp(const Imu& pre, const Imu& post, const fp& alpha);

  /**
   * @brief Mean propagation for the MSCEqF
   *
   * @param X Actual state estimate
   * @param xi0 Origin
   * @param u IMU measurement to propagate with
   * @param dt Delta time between input measurements
   */
  void propagateMean(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt);

  /**
   * @brief Covariance propagation for the MSCEqF
   *
   * @param X Actual state estimate
   * @param xi0 Origin
   * @param u IMU measurement to propagate the covariance with (used to compute state matrix)
   * @param dt Delta time between input measurements
   */
  void propagateCovariance(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt);

  /**
   * @brief This function computes the continuous-time state matrix A.
   * The state matrix A is the state matrix for the Dd element of the MSCEqF state, and for the
   * E element if extrinsic calibration is enabled.
   *
   * @param X Actual state estimate
   * @param xi0 Origin
   * @param u IMU measurement to propagate the covariance with (used to compute state matrix)
   * @return The state matrix A
   */
  const MatrixX stateMatrix(MSCEqFState& X, const SystemState& xi0, const Imu& u) const;

  /**
   * @brief This function computes the continuous-time input matrix B.
   *
   * @param X Actual state estimate
   * @param xi0 Origin
   * @return Discrete time process noise covariance matrix
   */
  const MatrixX inputMatrix(MSCEqFState& X, const SystemState& xi0) const;

  /**
   * @brief This function computes the discrete-time Matrix H which is then used to compute
   * the core state transition matrix as wel as the discrete time noise covariance matrix.
   * Based on the given state_transition_order_, this function returns either
   * the first order truncation, or the numerical computation of the matrix exponential.
   * Note that the core state transition matrix is defined as the  state transition matrix
   * for the Dd element of the MSCEqF state, and for the E element if extrinsic calibration is enabled.
   *
   * @param A Continuous time state matrix A
   * @param B Continuous time input matrix B
   * @param dt Delta time between input measurements
   * @return The H matrix which include the core state transition matrix and the discrete time input matrix
   *
   * @note This function is based on the following paper: Axelsson, Patrik. (2015). Discrete-Time Solutions to the
   * Continuous-Time Differential Lyapunov Equation With Applications to Kalman Filtering. Automatic Control, IEEE
   * Transactions on. 60. 632-643. 10.1109/TAC.2014.2353112.
   */
  const MatrixX discreteTimeMatrix(const MatrixX& A, const MatrixX& B, const fp& dt) const;

  ImuBuffer imu_buffer_;  //!< The imu measurement buffer
  Matrix12 Q_;            //!< The continuous time process noise covariance

  int state_transition_order_;  //!< Truncation order of the state transition matrix
  uint imu_buffer_max_size_;    //!< Maximum imu buffer size

  static constexpr fp eps_ = 1e-6;  //!< epsilon, minimum time difference accepted
};

}  // namespace msceqf

#endif  // PROPAGATOR_HPP