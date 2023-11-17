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

#include <map>
#include <variant>

#include "msceqf/system/system.hpp"
#include "msceqf/options/msceqf_options.hpp"
#include "msceqf/state/state_elements.hpp"

namespace msceqf
{
/**
 * @brief this class represent the state of the MSCEqF.
 * This includes the state of the lifted system (element of the symmetry group) and the covariance.
 *
 */
class MSCEqFState
{
 public:
  using MSCEqFStateKey = std::variant<MSCEqFStateElementName, uint>;  //!< Key to access the msceqf state map
  using MSCEqFKey = std::variant<MSCEqFStateKey, fp>;                 //!< Key to access the msceqf state and clones map

  using MSCEqFStateMap = std::unordered_map<MSCEqFStateKey, MSCEqFStateElementSharedPtr>;  //!< MSCEqF state map
  using MSCEqFClonesMap = std::map<fp, MSCEqFStateElementSharedPtr>;                       //!< MSCEqF clones map

  /**
   * @brief Deleted default constructor
   */
  MSCEqFState() = delete;

  /**
   * @brief Construct the state of the MSCEqF given the options
   *
   * @param opts Options of the MSCEqF state
   * @param xi0 Origin
   *
   * @note The MSCEqF state has not to be confused with the system state. The former is the state of the lifted system,
   * in which the EqF is based on, while latter is the state of the system posed on the homogenous space.
   * The given origin is needed to initialize cross-correlation whenever the origin is not identity.
   */
  MSCEqFState(const StateOptions& opts, const SystemState& xi0);

  /// Rule of Five
  MSCEqFState(const MSCEqFState& other);
  MSCEqFState(MSCEqFState&& other) noexcept;
  MSCEqFState& operator=(const MSCEqFState& other);
  MSCEqFState& operator=(MSCEqFState&& other) noexcept;
  ~MSCEqFState();

  /**
   * @brief Get a reference to the SE23 component of the Semi Direct Bias Group element of the MSCEqF state
   *
   * @return SE23 group element of the MSCEqF state
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE23& D() const;

  /**
   * @brief Get a copy of the SE3 component of the Semi Direct Bias Group element of the MSCEqF state that includes
   * the rotational component (R) and the first isometry (v)
   *
   * @return SE3 group element representing the rotational component and the first isometry of the MSCEqF state
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3 B() const;

  /**
   * @brief Get a copy of to the SE3 component of the Semi Direct Bias Group element of the MSCEqF state that includes
   * the rotational component (R) and the second isometry (p)
   *
   * @return SE3 group element representing the rotational component and the second isometry of the MSCEqF state
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3 C() const;

  /**
   * @brief Get a reference to the R6 component of the Semi Direct Bias Group element of the MSCEqF state
   *
   * @return R6 group element of the MSCEqF state
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const Vector6& delta() const;

  /**
   * @brief Get a reference to the SE3 element of the MSCEqF state
   *
   * @return SE3 group element of the MSCEqF state representing the element acting on the extrinsic calibration
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3& E() const;

  /**
   * @brief Get a reference to the In element of the MSCEqF state
   *
   * @return In group element of the MSCEqF state representing the element acting on the intrinsic calibration
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const In& L() const;

  /**
   * @brief Get a reference to the SOT3 element of the MSCEqF state that correspond to the given feature id
   *
   * @param feat_id Feature id
   * @return SOT3 group element of the MSCEqF state representing the element acting on the feature
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SOT3& Q(const uint& feat_id) const;

  /**
   * @brief Get a reference to the SE3 element of the MSCEqF clones that correspond to the given timestamp
   *
   * @param timestamp Timestamp
   * @return SE3 group element of the MSCEqF clones representing the element acting on the extrinsic calibration
   *
   * @note This function does not introduce any runtime overhead due to casting, because it uses static_pointer_cast
   */
  [[nodiscard]] const SE3& clone(const fp& timestamp) const;

  /**
   * @brief Get a reference to the index of the state element or the clone element corresponding to the given key
   *
   * @param key State element name, feature id or timestamp of clone
   * @return Index of the state element or the clone element corresponding to the given key
   */
  [[nodiscard]] const uint& index(const MSCEqFKey& key) const;

  /**
   * @brief Get a reference to the dof of the state element or the clone element corresponding to the given key
   *
   * @param key State element name, feature id or timestamp of clone
   * @return Degree of freedom of the state element or the clone element corresponding to the given key
   */
  [[nodiscard]] const uint& dof(const MSCEqFKey& key) const;

  /**
   * @brief Get the amount of clones
   *
   * @return Number of clones
   */
  [[nodiscard]] inline size_t clonesSize() const { return clones_.size(); }

  /**
   * @brief Get the timestamp of the clone to marginalize.
   * We implement our keyframing strategy here. So far we simply marginalize the oldest clone.
   *
   * @return Timestamp of the clone to marginalize
   */
  [[nodiscard]] const fp& cloneTimestampToMarginalize() const;

  /**
   * @brief Get a reference to the covariance matrix
   *
   * @return Covariance matrix
   */
  [[nodiscard]] const MatrixX& cov() const;

  /**
   * @brief get a constant copy of the covariance block relative to the elements (states or clones) corresponding to the
   * given keys.
   *
   * @param key State element name, feature id or timestamp of clone
   * @return Block of the covariance matrix corresponding to the given key
   */
  [[nodiscard]] const MatrixX covBlock(const MSCEqFKey& key) const;

  /**
   * @brief Get a constant copy of the the covariance submatrix (including cross-correlations) constructed with
   * covariance blocks relative to the elements (states or clones) corresponding to the given keys.
   * *The ordering of the covariance returned follows the ordering of the given keys.*
   *
   * @param keys Vector of state elements name, feature id or timestamp of clone
   * @return Stacked blocks of the covariance matrix corresponding to the given keys including cross-correlations
   */
  [[nodiscard]] const MatrixX subCov(const std::vector<MSCEqFKey>& keys) const;

  /**
   * @brief Get a constant copy of the the covariance submatrix (including cross-correlations) constructed with
   * covariance columns relative to the elements (states or clones) corresponding to the given keys.
   * *The ordering of the covariance returned follows the ordering of the given keys.*
   *
   * @param keys Vector of state elements name, feature id or timestamp of clone
   * @return Stacked columns of the covariance matrix corresponding to the given keys including cross-correlations
   */
  [[nodiscard]] const MatrixX subCovCols(const std::vector<MSCEqFKey>& keys) const;

  /**
   * @brief Get the state options
   *
   * @return State options
   */
  [[nodiscard]] inline const StateOptions& opts() const { return opts_; }

  /**
   * @brief Initialize MSCEqF state element into the state map, and the relative covariance block.
   *
   * @param key State element name or feature id
   * @param cov_block Corresponding blcok of the covariance matrix
   *
   * @note Note that the MSCEqF states are always initialized at the identity. This is correct since is xi0 that
   * "controls" how close to ground truth these are.
   */
  void initializeStateElement(const MSCEqFStateKey& key, const MatrixX& cov_block);

  /**
   * @brief Augment the MSCEqF clones map with a new clone of the actual E element of the MSCEqF state.
   * The new clone is mapped via the given timestamp
   *
   * @param timestamp Timestamp of the new clone
   */
  void stochasticCloning(const fp& timestamp);

  /**
   * @brief Marginalize out clone at a given timestamp
   *
   * @param timestamp Timestamp of the clone to marginalize
   */
  void marginalizeCloneAt(const fp& timestamp);

  /**
   * @brief Get a string describing the given MSCEqFStateKey
   *
   * @param key State element name or feature id
   * @return Key as string
   */
  static std::string toString(const MSCEqFStateKey& key);

  /**
   * @brief Return a random MSCEqF state without changing *this.
   * This method *WILL NOT* change the actual values of the state.
   * This method *WILL NOT* initialize the covariance or the clones map for the returned state.
   * This method will only initialize the state map with random values for the returned state.
   *
   * @return MSCEqF state
   *
   * @note *THIS IS MEANT TO BE AN HELPER FUNCTION FOR DEBUG/TESTING*
   */
  [[nodiscard]] const MSCEqFState Random() const;

  /**
   * @brief operator* overloading for MSCEqFState.
   * This function will perform the composition this * other for each element of the state map.
   * This method will *NOT* perform any composition for the covariance matrix, and for the clones map.
   *
   * @param other MSCEqF state
   * @return MSCEqF state
   *
   * @note *THIS IS MEANT TO BE AN HELPER FUNCTION FOR DEBUG/TESTING*
   */
  [[nodiscard]] const MSCEqFState operator*(const MSCEqFState& other) const;

 private:
  /**
   * @brief Preallocate space on the MSCEqF state map and clones_map based on given options
   */
  void preallocate();

  /**
   * @brief Insert given pointer into the MSCEqF state map and check that the pointer is not null.
   *
   * @param key State element name or feature id
   * @param ptr Pointer to MSCEqF state element
   * @return true if the element has been succesfully inserted, false if a corresponding key existed already
   */
  [[nodiscard]] bool insertStateElement(const MSCEqFStateKey& key, MSCEqFStateElementUniquePtr ptr);

  /**
   * @brief Insert given pointer into the MSCEqF clones map and check that the pointer is not null.
   *
   * @param timestamp Clone timestamp
   * @param ptr Pointer to MSCEqF clone element
   * @return true if the element has been succesfully inserted, false if a corresponding key existed already
   */
  [[nodiscard]] bool insertCloneElement(const fp& timestamp, MSCEqFStateElementUniquePtr ptr);

  /**
   * @brief Get the MSCEqF element (state or clone) pointer (base) given the key
   *
   * @param key State element name, feature id or timestamp of clone
   * @return Pointer to state element
   */
  [[nodiscard]] const MSCEqFStateElementSharedPtr& getPtr(const MSCEqFKey& key) const;

  friend class Symmetry;             //!< Symmetry can access private members of MSCEqFState
  friend class Propagator;           //!< Propagator can access private members of MSCEqFState
  friend class Updater;              //!< Updater can access private members of MSCEqFState
  friend class ZeroVelocityUpdater;  //!< Zero velocity updater can access private members of MSCEqFState

  StateOptions opts_;  //!< State Options

  MatrixX cov_;             //!< MSCEqF State covariance (Sigma matrix)
  MSCEqFStateMap state_;    //!< MSCEqF State elements mapped by their names
  MSCEqFClonesMap clones_;  //!< MSCEqF Stochastic clones mapped by their timestamps
};

}  // namespace msceqf

#endif  // STATE_HPP
