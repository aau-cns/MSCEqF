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

#ifndef STATE_HPP
#define STATE_HPP

#include <variant>

#include "msceqf/msceqf_options.hpp"
#include "msceqf/state/state_elements.hpp"

namespace msceqf
{
/**
 * @brief this class represent the state of the MSCEqF.
 * This includes the state of the lifted system (element of the symmetry group) and the covariance.
 *
 * @note
 */
class MSCEqFState
{
 public:
  using MSCEqFStateKey = std::variant<MSCEqFStateElementName, uint>;  //!< Key to access the msceqf state map
  using MSCEqFStateMap = std::unordered_map<MSCEqFStateKey, MSCEqFStateElementSharedPtr>;  //!< MSCEqF state map
  using MSCEqFClonesMap = std::unordered_map<fp, MSCEqFSE3StateSharedPtr>;                 //!< MSCEqF clones map

  /**
   * @brief Deleted default constructor
   */
  MSCEqFState() = delete;

  /**
   * @brief Construct the state of the MSCEqF given the options
   *
   * @param opts options of the MSCEqF state
   *
   * @note The MSCEqF state has not to be confused with the system state. The former is the state of the lifted system,
   * in which the EqF is based on, while latter is the state of the system posed on the homogenous space.
   */
  MSCEqFState(const StateOptions& opts);

  /// Rule of Five
  MSCEqFState(const MSCEqFState& other);
  MSCEqFState(MSCEqFState&& other) noexcept;
  MSCEqFState& operator=(const MSCEqFState& other);
  MSCEqFState& operator=(MSCEqFState&& other) noexcept;
  ~MSCEqFState();

  /**
   * @brief Get a reference to the SE23 component of the Semi Direct Bias Group element of the MSCEqF state
   *
   * @return const SE23&
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE23& D() const;

  /**
   * @brief Get a copy of the SE3 component of the Semi Direct Bias Group element of the MSCEqF state that includes
   * the rotational component (R) and the first isometry (v)
   *
   * @return const SE3
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3 B() const;

  /**
   * @brief Get a copy of to the SE3 component of the Semi Direct Bias Group element of the MSCEqF state that includes
   * the rotational component (R) and the second isometry (p)
   *
   * @return const SE3
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3 C() const;

  /**
   * @brief Get a reference to the R6 component of the Semi Direct Bias Group element of the MSCEqF state
   *
   * @return const Vector6&
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const Vector6& delta() const;

  /**
   * @brief Get a reference to the SE3 element of the MSCEqF state
   *
   * @return const SE3&
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3& E() const;

  /**
   * @brief Get a reference to the In element of the MSCEqF state
   *
   * @return const In&
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const In& L() const;

  /**
   * @brief Get a reference to the SOT3 element of the MSCEqF state that correspond to the given feature id
   *
   * @param feat_id feature id
   * @return const SOT3&
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SOT3& Q(const uint& feat_id) const;

  /**
   * @brief Get a reference to the covariance matrix
   *
   * @return const MatrixX&
   */
  [[nodiscard]] const MatrixX& Cov() const;

  /**
   * @brief get a constant copy of the covariance block relative to the element corresponding to the given key
   *
   * @param key state element name or feature id
   * @return const MatrixX
   */
  [[nodiscard]] const MatrixX CovBlock(const MSCEqFStateKey& key) const;

  /**
   * @brief Initialize MSCEqF state element into the state map, and the relative covariance block.
   *
   * @param key state element name or feature id
   * @param cov_block corresponding element of the covariance matrix
   *
   * @note Note that the MSCEqF states are always initialized at the identity. This is correct since is xi0 that
   * "controls" how close to ground truth these are.
   */
  void initializeStateElement(const MSCEqFStateKey& key, const MatrixX& cov_block);

 private:
  /**
   * @brief Insert given pointer into the MSCEqF state map and check that the pointer is not null.
   *
   * @param key state element name or feature id
   * @param ptr pointer to MSCEqF state element
   */
  void insertStateElement(const MSCEqFStateKey& key, MSCEqFStateElementUniquePtr ptr);

  /**
   * @brief Get the MSCEqF element pointer (base) given the key
   *
   * @param key
   * @return const MSCEqFStateElementSharedPtr&
   */
  [[nodiscard]] const MSCEqFStateElementSharedPtr& getPtr(const MSCEqFStateKey& key) const;

  MatrixX cov_;             //!< MSCEqF State covariance (Sigma matrix)
  MSCEqFStateMap state_;    //!< MSCEqF State elements mapped by their names
  MSCEqFClonesMap clones_;  //!< MSCEqF Stochastic clones mapped by their timestamps

 public:
  StateOptions opts_;  //!< MSCEqF Options
};

// [TODO] Insert clone method

// [TODO] Method to get covariance for a subset of variables. --> Do i need this? is it necessary to update as OV does?

// [TODO] Marginalize method

// [TODO] Delayed feature init

// [COMMENT] The choice of the unorderd map is due to fast retreiving of elements (hash table)

// [COMMENT] When should I initialize xi0? probably toghether with the state but from the MSCEqF manager... I probably
// should not mix it in

}  // namespace msceqf

#endif  // STATE_HPP