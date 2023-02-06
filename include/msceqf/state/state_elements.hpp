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
  virtual ~MSCEqFStateElement() = default;

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
   * @brief update function to update the value of the state element by right multiplication
   *
   * @param delta delta value to update with
   */
  virtual void updateRight(const VectorX& delta) = 0;

  /**
   * @brief update function to update the value of the state element by left multiplication
   *
   * @param delta delta value to update with
   */
  virtual void updateLeft(const VectorX& delta) = 0;

  /**
   * @brief Clone
   *
   * @return std::unique_ptr<MSCEqFStateElement>
   */
  virtual std::unique_ptr<MSCEqFStateElement> clone() const = 0;

 protected:
  /// Rule of Five
  MSCEqFStateElement() = delete;
  MSCEqFStateElement(const MSCEqFStateElement&) = default;
  MSCEqFStateElement(MSCEqFStateElement&&) = default;
  MSCEqFStateElement& operator=(const MSCEqFStateElement&) = default;
  MSCEqFStateElement& operator=(MSCEqFStateElement&&) = default;

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
 * @brief This struct represent the Semi Direct bias state of the MSCEqF
 *
 */
struct MSCEqFSDBState final : public MSCEqFStateElement
{
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
   * @brief Update the Semi Direct Bias element of the state by right multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateRight(const VectorX& delta) override { Dd_.multiplyRight(SDB::exp(delta)); }

  /**
   * @brief Update the Semi Direct Bias element of the state by left multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateLeft(const VectorX& delta) override { Dd_.multiplyLeft(SDB::exp(delta)); }

  /**
   * @brief Clone the Semi Direct bias (SDB) element of state of the MSCEqF
   *
   * @return std::unique_ptr<MSCEqFStateElement>
   */
  std::unique_ptr<MSCEqFStateElement> clone() const override { return std::make_unique<MSCEqFSDBState>(*this); }

  SDB Dd_;  //!< The Semi Direct Bias element of the state
};

/**
 * @brief This struct represent the Special Euclidean Group of dimension 3 state of the MSCEqF
 *
 */
struct MSCEqFSE3State final : public MSCEqFStateElement
{
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
   * @brief Update the Special Euclidean Group element of the state by right multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateRight(const VectorX& delta) override { E_.multiplyRight(SE3::exp(delta)); }

  /**
   * @brief Update the Special Euclidean Group element of the state by left multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateLeft(const VectorX& delta) override { E_.multiplyLeft(SE3::exp(delta)); }

  /**
   * @brief Clone the Special Euclidean Group (SE3) element of state of the MSCEqF
   *
   * @return std::unique_ptr<MSCEqFStateElement>
   */
  std::unique_ptr<MSCEqFStateElement> clone() const override { return std::make_unique<MSCEqFSE3State>(*this); }

  SE3 E_;  //!< The Special Euclidean element of the state
};

/**
 * @brief This struct represent the Intrinsic state of the MSCEqF
 *
 */
struct MSCEqFInState final : public MSCEqFStateElement
{
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
   * @brief Update the Intrinsic element of the state by right multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateRight(const VectorX& delta) override { L_.multiplyRight(In::exp(delta)); }

  /**
   * @brief Update the Intrinsic element of the state by left multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateLeft(const VectorX& delta) override { L_.multiplyLeft(In::exp(delta)); }

  /**
   * @brief Clone the Special Intrinsic (In) element of state of the MSCEqF
   *
   * @return std::unique_ptr<MSCEqFStateElement>
   */
  std::unique_ptr<MSCEqFStateElement> clone() const override { return std::make_unique<MSCEqFInState>(*this); }

  In L_;  //!< The Intrinsic element of the state
};

/**
 * @brief This struct represent the Scaled Orthogonal Transforms state of the MSCEqF
 *
 */
struct MSCEqFSOT3State final : public MSCEqFStateElement
{
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
   * @brief Update the Scaled Orthogonal Transforms element of the state by right multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateRight(const VectorX& delta) override { Q_.multiplyRight(SOT3::exp(delta)); }

  /**
   * @brief Update the Scaled Orthogonal Transforms element of the state by left multiplication
   *
   * @param delta delta vector to update the state element with
   */
  void updateLeft(const VectorX& delta) override { Q_.multiplyLeft(SOT3::exp(delta)); }

  /**
   * @brief Clone the Scaled Orthogonal Transforms (SOT3) element of state of the MSCEqF
   *
   * @return std::unique_ptr<MSCEqFStateElement>
   */
  std::unique_ptr<MSCEqFStateElement> clone() const override { return std::make_unique<MSCEqFSOT3State>(*this); }

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
  if constexpr (std::is_base_of_v<MSCEqFStateElement, T>)
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