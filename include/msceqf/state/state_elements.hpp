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

#ifndef STATE_ELEMENTS_HPP
#define STATE_ELEMENTS_HPP

#include <memory>

#include "types/fptypes.hpp"

namespace msceqf
{
/**
 * @brief This enum class define the names of the MSCEqF state elements.
 * This is used to create a map of state element mapped by the name,
 * and retreive specific element of the state through a map.
 *
 * @note Note that SOT3 ekement is excluded since multiple SOT3 elements will be refeered to through the features id
 */
enum class MSCEqFStateElementName
{
  Dd,  //!< Name of the Semi Direct Bias element of the MSCEqF state
  E,   //!< Name of the Special Euclidean element of the MSCEqF state
  L,   //!< Name of the Intrinsic element of the MSCEqF state
};

/**
 * @brief This class represent the base class for a general element of the MSCEqF state.
 * This include the index of the variable (index in the residual, and in the covariance), and the degrees of
 * freedom.
 *
 * @note Type-based index system inspired by: https://ieeexplore.ieee.org/document/9196524
 */
class MSCEqFStateElement
{
 public:
  /**
   * @brief Destroy the MSCEqFStateElement object
   *
   */
  virtual ~MSCEqFStateElement(){};

  /**
   * @brief Get the starting index of the state element in the residual, and in the covariance
   *
   * @return int
   */
  [[nodiscard]] int getIndex() { return idx_; }

  /**
   * @brief Get the degrees of freedom of the state element (dimension of relative covariance and residual block)
   *
   * @return int
   */
  [[nodiscard]] int getDof() { return dof_; }

  /**
   * @brief update function to update the state element
   *
   * @param delta scaled residual
   */
  virtual void update(const VectorX& delta) = 0;

 protected:
  /**
   * @brief Deleted default constructor
   *
   */
  MSCEqFStateElement() = delete;

  /**
   * @brief Construct a MSCEqFStateElement object
   *
   * @param idx  Starting index of the element in the residual, and in the covariance
   * @param dof Degrees of freedom of hte element (dimension of relative covariance and residual block)
   */
  MSCEqFStateElement(const uint& idx, const uint& dof) : idx_(idx), dof_(dof){};

  uint idx_;  //!< Starting index of the element in the residual, and in the covariance
  uint dof_;  //!< Degrees of freedom of hte element (dimension of relative covariance and residual block)
};

/**
 * @brief This class represent the Semi Direct bias state of the MSCEqF
 *
 */
class MSCEqFSDBState final : public MSCEqFStateElement
{
 public:
  /**
   * @brief Deleted default constructor
   *
   */
  MSCEqFSDBState() = delete;

  /**
   * @brief Construct an identity MSCEqFSDBState object
   *
   * @param idx starting index of the variable in the covariance
   */
  MSCEqFSDBState(const uint& idx) : MSCEqFStateElement(idx, 15), Dd_(){};

  /**
   * @brief Update the Semi Direct Bias element of the state
   *
   * @param delta scaled residual
   */
  void update(const VectorX& delta) override { Dd_ *= SDB::exp(delta); }

  /**
   * @brief Get a reference to the SDB element
   *
   * @return const SDB&
   */
  [[nodiscard]] const SDB& getDd() { return Dd_; }

 private:
  SDB Dd_;  //!< The Semi Direct Bias element of the state
};

/**
 * @brief This class represent the Special Euclidean Group of dimension 3 state of the MSCEqF
 *
 */
class MSCEqFSE3State final : public MSCEqFStateElement
{
 public:
  /**
   * @brief Deleted default constructor
   *
   */
  MSCEqFSE3State() = delete;

  /**
   * @brief Construct an identity MSCEqFSE3State object
   *
   * @param idx starting index of the variable in the covariance
   */
  MSCEqFSE3State(const uint& idx) : MSCEqFStateElement(idx, 6), E_(){};

  /**
   * @brief Update the Special Euclidean Group of dimension 3 element of the state
   *
   * @param delta scaled residual
   */
  void update(const VectorX& delta) override { E_ *= SE3::exp(delta); }

  /**
   * @brief Get a reference to the SE3 element
   *
   * @return const SE3&
   */
  [[nodiscard]] const SE3& getE() { return E_; }

 private:
  SE3 E_;  //!< The Special Euclidean element of the state
};

/**
 * @brief This class represent the Intrinsic state of the MSCEqF
 *
 */
class MSCEqFInState final : public MSCEqFStateElement
{
 public:
  /**
   * @brief Deleted default constructor
   *
   */
  MSCEqFInState() = delete;

  /**
   * @brief Construct an identity MSCEqFInState object
   *
   * @param idx starting index of the variable in the covariance
   */
  MSCEqFInState(const uint& idx) : MSCEqFStateElement(idx, 4), L_(){};

  /**
   * @brief Update the Intrinsic element of the state
   *
   * @param delta scaled residual
   */
  void update(const VectorX& delta) override { L_ *= In::exp(delta); }

  /**
   * @brief Get a reference to the In element
   *
   * @return const In&
   */
  [[nodiscard]] const In& getL() { return L_; }

 private:
  In L_;  //!< The Intrinsic element of the state
};

/**
 * @brief This class represent the Scaled Orthogonal Transforms state of the MSCEqF
 *
 */
class MSCEqFSOT3State final : public MSCEqFStateElement
{
 public:
  /**
   * @brief Deleted default constructor
   *
   */
  MSCEqFSOT3State() = delete;

  /**
   * @brief Construct an identity MSCEqFSOT3State object
   *
   * @param idx starting index of the variable in the covariance
   */
  MSCEqFSOT3State(const uint& idx) : MSCEqFStateElement(idx, 4), Q_(){};

  /**
   * @brief Update the Scaled Orthogonal Transforms element of the state
   *
   * @param delta scaled residual
   */
  void update(const VectorX& delta) override { Q_ *= SOT3::exp(delta); }

  /**
   * @brief Get a reference to the SOT3 element
   *
   * @return const SOT3&
   */
  [[nodiscard]] const SOT3& getQ() { return Q_; }

 private:
  SOT3 Q_;  //!< The Scaled Orthogonal Transforms element of the state
};

using MSCEqFStateElementSharedPtr = std::shared_ptr<MSCEqFStateElement>;
using MSCEqFStateElementUniquePtr = std::unique_ptr<MSCEqFStateElement>;
using MSCEqFSDBStateSharedPtr = std::shared_ptr<MSCEqFSDBState>;
using MSCEqFSDBStateUniquePtr = std::unique_ptr<MSCEqFSDBState>;
using MSCEqFSE3StateSharedPtr = std::shared_ptr<MSCEqFSE3State>;
using MSCEqFSE3StateUniquePtr = std::unique_ptr<MSCEqFSE3State>;
using MSCEqFInStateSharedPtr = std::shared_ptr<MSCEqFInState>;
using MSCEqFInStateUniquePtr = std::unique_ptr<MSCEqFInState>;
using MSCEqFSOT3StateSharedPtr = std::shared_ptr<MSCEqFSOT3State>;
using MSCEqFSOT3StateUniquePtr = std::unique_ptr<MSCEqFSOT3State>;

/**
 * @brief Factory method for state elements
 *
 * @tparam underlying type of pointer to be made
 * @param idx starting index of the element in the residual, and in the covariance
 * @return std::unique_ptr<MSCEqFStateElement>
 */
template <typename T>
[[nodiscard]] static MSCEqFStateElementUniquePtr createMSCEqFStateElement(const uint& idx)
{
  if constexpr (std::is_constructible_v<T, decltype(idx)>)
  {
    return std::make_unique<T>(idx);
  }
  else
  {
    return nullptr;
  }
}

}  // namespace msceqf

#endif  // STATE_ELEMENTS_HPP

// [COMMENT] Data oriented design -> Object with vector faster than vector of objects --> Is it applicable to the Qs?