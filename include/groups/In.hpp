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

#ifndef IN_HPP
#define IN_HPP

#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace group
{
/**
 * @brief The Intrinsic Group. This represent the group describing camera intrinsics transformation.
 *
 * @tparam FPType. Floating point type (float, double, long double)
 */
template <typename FPType>
class In
{
 public:
  using MatrixType = Eigen::Matrix<FPType, 3, 3>;   //!< The underlying 3x3 group matrix type
  using TMatrixType = Eigen::Matrix<FPType, 4, 4>;  //!< The underlying 4x4 transformation matrix type
  using VectorType = Eigen::Matrix<FPType, 4, 1>;   //!< The underlying R4 vector type
  using Vector3Type = Eigen::Matrix<FPType, 3, 1>;  //!< The underlying R3 vector type

  /**
   * @brief Construct an identity In Group object
   */
  In() : fx_(1.0), fy_(1.0), cx_(0.0), cy_(0.0){};

  /**
   * @brief Construct a In Group object given the intrinsics
   *
   * @param L const Eigen::Matrix<FPType, 4, 1>&
   */
  In(const FPType& fx, const FPType& fy, const FPType& cx, const FPType& cy) : fx_(fx), fy_(fy), cx_(cx), cy_(cy){};

  /**
   * @brief Construct a In Group object given a matrix (camera intrinsic matrix)
   *
   * @param L const Eigen::Matrix<FPType, 3, 3>&
   */
  In(const MatrixType& L) : fx_(1.0), fy_(1.0), cx_(0.0), cy_(0.0)
  {
    fx_ = L(0, 0);
    fy_ = L(1, 1);
    cx_ = L(0, 2);
    cy_ = L(1, 2);
  };

  /**
   * @brief Construct a In Group object given a vector <fx, fy, cx, cy>
   *
   * @param L const Eigen::Matrix<FPType, 4, 1>&
   */
  In(const VectorType& l) : fx_(1.0), fy_(1.0), cx_(0.0), cy_(0.0)
  {
    fx_ = l(0);
    fy_ = l(1);
    cx_ = l(2);
    cy_ = l(3);
  };

  /**
   * @brief wedge operator, transform a vector in R4 to a matrix in the Lie Algebra of In
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   * @return const Eigen::Matrix<FPType, 3, 3>
   */
  [[nodiscard]] static const MatrixType wedge(const VectorType& u)
  {
    MatrixType U;
    U << u(0), 0.0, u(2), 0.0, u(1), u(3), 0.0, 0.0, 0.0;
    return (U);
  }

  /**
   * @brief transform a matrix in the Lie Algebra of In to a vector in R4
   *
   * @param U const Eigen::Matrix<FPType, 3, 3>&
   *
   * @return const Eigen::Matrix<FPType, 4, 1>
   */
  [[nodiscard]] static const VectorType vee(const MatrixType& U)
  {
    VectorType u;
    u << U(0, 0), U(1, 1), U(0, 2), U(1, 2);
    return (u);
  }

  /**
   * @brief so3 adjoint matrix
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] static const TMatrixType adjoint(const VectorType& u)
  {
    TMatrixType ad;
    ad << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -u(2), 0.0, u(0), 0.0, 0.0, -u(3), 0.0, u(1);
    return ad;
  }

  /**
   * @brief The exponential map for the In Group.
   * Returns a In object given a vector u in R4 (equivalent to exp(wedge(u)))
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const In
   */
  [[nodiscard]] static const In exp(const VectorType& u)
  {
    MatrixType L = wedge(u).exp();
    return In(L);
  }

  /**
   * @brief The logarithmic map for the In Group.
   * Return a vector given a In object (equivalent to vee(log(X)))
   *
   * @param X const In&
   *
   * @return const Eigen::Matrix<FPType, 4, 1>
   */
  [[nodiscard]] static const VectorType log(const In& X) { return vee(X.K().log()); }

  /**
   * @brief get a constant copy of the inverse of the In object
   *
   * @return const In
   */
  [[nodiscard]] const In inv() const
  {
    return In(FPType(1.0 / fx_), FPType(1.0 / fy_), FPType(-cx_ / fx_), FPType(-cy_ / fy_));
  }

  /**
   * @brief Get the matrix representation of In
   *
   * @return const Eigen::Matrix<FPType, 3, 3>&
   */
  [[nodiscard]] const MatrixType K() const
  {
    MatrixType L;
    L << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
    return L;
  }

  /**
   * @brief Get the matrix representation of In
   *
   * @return const Eigen::Matrix<FPType, 3, 3>&
   */
  [[nodiscard]] const MatrixType asMatrix() const { return K(); }

  /**
   * @brief In Adjoint matrix
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] const TMatrixType Adjoint() const
  {
    TMatrixType Ad;
    Ad << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -cx_, 0.0, fx_, 0.0, 0.0, -cy_, 0.0, fy_;
    return Ad;
  }

  /**
   * @brief In Inverse Adjoint matrix
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] const TMatrixType invAdjoint() const
  {
    TMatrixType invAd;
    invAd << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, cx_ / fx_, 0.0, 1.0 / fx_, 0.0, 0.0, cy_ / fy_, 0.0, 1.0 / fy_;
    return invAd;
  }

  /**
   * @brief Operator * overloading.
   * Implements the In composition this * other
   *
   * @param other const In&
   *
   * @return const In
   *
   * @note usage: z = x * y
   */
  [[nodiscard]] const In operator*(const In& other) const
  {
    return In(fx_ * other.fx_, fy_ * other.fy_, fx_ * other.cx_ + cx_, fy_ * other.cy_ + cy_);
  }

  /**
   * @brief Operator * overloading.
   * Implements the In composition with a Lie algebra of In element this * other
   *
   * @param other Eigen::Matrix<FPType, 3, 3>
   *
   * @return const Eigen::Matrix<FPType, 3, 3>
   */
  [[nodiscard]] const MatrixType operator*(const MatrixType& other) const { return K() * other; }

  /**
   * @brief Operator * overloading.
   * Implements the In action on a R3 vector
   *
   * @param other const Eigen__Matrix<FPType, 3, 1>&
   *
   * @return const Eigen__Matrix<FPType, 3, 1>
   */
  [[nodiscard]] const Vector3Type operator*(const Vector3Type& other) const
  {
    Vector3Type x;
    x << fx_ * other(0) + cx_ * other(2), fy_ * other(1) + cy_ * other(2), other(2);
    return x;
  }

  /**
   * @brief Implements the In composition this = this * other
   *
   * @param other const In&
   *
   * @return const In&
   */
  const In& multiplyRight(const In& other)
  {
    cx_ = fx_ * other.cx_ + cx_;
    cy_ = fy_ * other.cy_ + cy_;
    fx_ *= other.fx_;
    fy_ *= other.fy_;
    return *this;
  }

  /**
   * @brief Implements the In composition this = other * this
   *
   * @param other const In&
   *
   * @return const In&
   */
  const In& multiplyLeft(const In& other)
  {
    cx_ = other.fx_ * cx_ + other.cx_;
    cy_ = other.fy_ * cy_ + other.cy_;
    fx_ *= other.fx_;
    fy_ *= other.fy_;
    return *this;
  }

 protected:
  FPType fx_;  //!< The x-direction focal length
  FPType fy_;  //!< The y-direction focal length
  FPType cx_;  //!< The x-direction center
  FPType cy_;  //!< The y-direction center

  static constexpr FPType eps_ = 1e-6;  //!< Epsilon
};

using Inf = In<float>;   //!< The Intrinsic group with single precision floating point
using Ind = In<double>;  //!< The Intrinsic group with double precision floating point

}  // namespace group

#endif  // IN_HPP