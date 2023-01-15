// Copyright (C) 2023 Alessandro Fornasier, Pieter van Goor.
// Control of Networked Systems, University of Klagenfurt, Austria.
// System Theory and Robotics Lab, Australian Centre for Robotic
// Vision, Australian national University, Australia.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>,
// <pieter.vangoor@anu.edu.au>.

#ifndef SOT3_HPP
#define SOT3_HPP

#include "SO3.hpp"

namespace group
{
/**
 * @brief The Scaled orthogonal transforms of dimension 3 (SOT3)
 *
 * @tparam FPType. Floating point type (float, double, long double)
 *
 * @note EqVIO: An Equivariant Filter for Visual Inertial Odometry [https://arxiv.org/abs/2205.01980]
 */
template <typename FPType>
class SOT3
{
 public:
  using SO3Type = SO3<FPType>;                      //!< The underlying SO3 type
  using VectorType = Eigen::Vector<FPType, 4>;      //!< R4 Vectorspace element type (isomorphic to Lie Algebra sot3)
  using MatrixType = Eigen::Matrix<FPType, 4, 4>;   //!< Lie Algebra / Lie Group matrix type
  using TMatrixType = Eigen::Matrix<FPType, 4, 4>;  //!< the transformation matrix type (Linear operator on R4)
  using ScaleType = FPType;                         //!< Scale factor type

  /**
   * @brief Construct an identity SOT3 object
   */
  SOT3() : C_(SO3Type()), s_(1.0) {}

  /**
   * @brief Construct a SOT3 object from a given normalized quaternion, and scale.
   *
   * @param q const Eigen::Quaternion<FPType>&
   * @param s const FPType&
   */
  SOT3(const typename SO3Type::QuaternionType& q, const ScaleType& s) : C_(q), s_(s) { checkScale(); }

  /**
   * @brief Construct a SOT3 object from a given rotation matrix, and scale.
   *
   * @param R const Eigen::Matrix<FPType, 3, 3>&
   * @param s const FPType&
   */
  SOT3(const typename SO3Type::MatrixType& R, const ScaleType& s) : C_(R), s_(s) { checkScale(); }

  /**
   * @brief Construct a SOT3 object from a given SO3 object, and scale.
   *
   * @param C const SO3<FPType>&
   * @param s const FPType&
   */
  SOT3(const SO3Type& C, const ScaleType& s) : C_(C), s_(s) { checkScale(); }

  /**
   * @brief Construct a SOT3 object from a given matrix.
   *
   * @param Q const Eigen::Matrix<FPType, 4, 4>&
   */
  SOT3(const MatrixType& Q) : C_(Q.template block<3, 3>(0, 0)), s_(Q(3, 3)) { checkScale(); }

  /**
   * @brief wedge operator, transform a vector in R4 to a matrix in sot3
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] static const MatrixType wedge(const VectorType& u)
  {
    MatrixType U = MatrixType::Zero();
    U.template block<3, 3>(0, 0) = SO3Type::wedge(u.template block<3, 1>(0, 0));
    U(3, 3) = u(3);
    return U;
  }

  /**
   * @brief transform a matrix in sot3 to a vector in R4
   *
   * @param U const Eigen::Matrix<FPType, 4, 4>&
   *
   * @return const Eigen::Matrix<FPType, 4, 1>
   */
  [[nodiscard]] static const VectorType vee(const MatrixType& U)
  {
    VectorType u = VectorType::Zero();
    u.template block<3, 1>(0, 0) = SO3Type::vee(U.template block<3, 3>(0, 0));
    u(3) = U(3, 3);
    return u;
  }

  /**
   * @brief sot3 adjoint matrix
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] static const TMatrixType adjoint(const VectorType& u)
  {
    TMatrixType ad = TMatrixType::Zero();
    ad.template block<3, 3>(0, 0) = SO3Type::wedge(u.template block<3, 1>(0, 0));
    return ad;
  }

  /**
   * @brief SOT3 left Jacobian matrix
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] static const TMatrixType leftJacobian(const VectorType& u)
  {
    TMatrixType J = TMatrixType::Identity();
    J.template block<3, 3>(0, 0) = SO3Type::leftJacobian(u.template block<3, 1>(0, 0));
    return J;
  }

  /**
   * @brief SOT3 right Jacobian matrix
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] static const TMatrixType rightJacobian(const VectorType& u) { return leftJacobian(-u); }

  /**
   * @brief The exponential map for SOT3.
   * Returns a SOT3 object given a vector u in R4 (equivalent to exp(wedge(u)))
   *
   * @param u const Eigen::Matrix<FPType, 4, 1>&
   *
   * @return const SOT3<FPType>
   */
  [[nodiscard]] static const SOT3 exp(const VectorType& u)
  {
    if (u(3) < eps_)
    {
      return SOT3(SO3Type::exp(u.template block<3, 1>(0, 0)), 1.0 + std::expm1(u(3)));
    }
    else
    {
      return SOT3(SO3Type::exp(u.template block<3, 1>(0, 0)), std::exp(u(3)));
    }
  }

  // /**
  //  * @brief The exponential map for SOT3.
  //  * Returns a SOT3 object given a vector U in sot3
  //  *
  //  * @param U const Eigen::Matrix<FPType, 4, 4>&
  //  *
  //  * @return const SOT3<FPType>
  //  */
  // [[nodiscard]]  static const SOT3 exp(const MatrixType& U) { return exp(vee(U)); }

  /**
   * @brief The logarithmic map for SOT3.
   * Return a vector given a SOT3 object (equivalent to vee(log(X)))
   *
   * @param X const SOT3<FPType>&
   *
   * @return const Eigen::Matrix<FPType, 4, 1>
   */
  [[nodiscard]] static const VectorType log(const SOT3& X)
  {
    VectorType u = VectorType::Zero();
    u.template block<3, 1>(0, 0) = SO3Type::log(X.C_);
    u(3) = std::log(X.s_);
    return u;
  }

  // /**
  //  * @brief The logarithmic map for SOT3.
  //  * Return a sot3 matrix given a SOT3 object
  //  *
  //  * @param X const SOT3<FPType>&
  //  *
  //  * @return const Eigen::Matrix<FPType, 4, 4>
  //  */
  // [[nodiscard]]  static const MatrixType log(const SOT3& X) { return wedge(log(X)); }

  /**
   * @brief Operator * overloading.
   * Implements the SOT3 composition this * other
   *
   * @param other const SOT3<FPType>&
   *
   * @return const SOT3<FPType>
   *
   * @note usage: z = x * y
   */
  [[nodiscard]] const SOT3 operator*(const SOT3& other) const { return SOT3(C_ * other.C_, s_ * other.s_); }

  /**
   * @brief Operator * overloading.
   * Implements the SOT3 composition with a sot3 element this * other
   *
   * @param other const Eigen::Matrix<FPType, 4, 4>&
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   *
   * @note usage: z = x * y
   */
  [[nodiscard]] const MatrixType operator*(const MatrixType& other) const
  {
    MatrixType res = MatrixType::Zero();
    res.template block<3, 3>(0, 0) = C_.R() * other.template block<3, 3>(0, 0);
    res(3, 3) = s_ * other(3, 3);
    return res;
  }

  /**
   * @brief Operator * overloading.
   * Implements the SOT3 action on a R3 vector
   *
   * @param other const Eigen::Matrix<FPType, 3, 1>&
   *
   * @return const Eigen::Matrix<FPType, 3, 1>
   */
  [[nodiscard]] const typename SO3Type::VectorType operator*(const typename SO3Type::VectorType& other) const
  {
    return s_ * (C_.R() * other);
  }

  /**
   * @brief Implements the SOT3 composition this = this * other
   *
   * @param other const SOT3<FPType>&
   *
   * @return const SOT3<FPType>&
   */
  const SOT3& multiplyRight(const SOT3& other)
  {
    C_.multiplyRight(other.C_);  // C_ * other.C_
    s_ *= other.s_;
    return *this;
  }

  /**
   * @brief Implements the SOT3 composition this = other * this
   *
   * @param other const SOT3<FPType>&
   *
   * @return const SOT3<FPType>&
   */
  const SOT3& multiplyLeft(const SOT3& other)
  {
    C_.multiplyLeft(other.C_);  // other.C_ * th.C_
    s_ *= other.s_;
    return *this;
  }

  /**
   * @brief get a constant copy of the inverse of the SOT3 object
   */
  [[nodiscard]] const SOT3 inv() const { return SOT3(C_.R().transpose(), 1 / s_); }

  /**
   * @brief Get a constant copy of the SOT3 object as a matrix
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] const MatrixType Q() const
  {
    MatrixType Q = MatrixType::Identity();
    Q.template block<3, 3>(0, 0) = C_.R();
    Q(3, 3) = s_;
    return Q;
  }

  /**
   * @brief Get a constant reference to the SOT3 rotation matrix
   *
   * @return const Eigen::Matrix<FPType, 3, 3>&
   */
  [[nodiscard]] const typename SO3Type::MatrixType& R() const { return C_.R(); }

  /**
   * @brief Get a constant reference to the SOT3 normalized quaternion
   *
   * @return const Eigen::Quaternion<FPType>&
   */
  [[nodiscard]] const typename SO3Type::QuaternionType& q() const { return C_.q(); }

  /**
   * @brief Get a constant reference to the SOT3 scale
   *
   * @return const FPType&
   */
  [[nodiscard]] const ScaleType& s() const { return s_; }

  /**
   * @brief Get a constant copy of the SOT3 object as a matrix
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] const MatrixType asMatrix() const { return Q(); }

  /**
   * @brief Set SOT3 object value from given matrix
   *
   * @param Q const Eigen::Matrix<FPType, 4, 4>&
   */
  void fromQ(const MatrixType& Q)
  {
    C_.fromR(Q.template block<3, 3>(0, 0));
    s_ = Q(3, 3);
    checkScale();
  }

  /**
   * @brief SOT3 Adjoint matrix
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] const TMatrixType Adjoint() const
  {
    TMatrixType Ad = TMatrixType::Identity();
    Ad.template block<3, 3>(0, 0) = C_.R();
    return Ad;
  }

  /**
   * @brief SOT3 Inverse Adjoint matrix
   *
   * @return const Eigen::Matrix<FPType, 4, 4>
   */
  [[nodiscard]] const TMatrixType invAdjoint() const
  {
    TMatrixType Ad = TMatrixType::Identity();
    Ad.template block<3, 3>(0, 0) = C_.R().transpose();
    return Ad;
  }

 protected:
  /**
   * @brief Check that s_ is grater than zero
   */
  void checkScale()
  {
    if (s_ < eps_)
    {
      throw std::invalid_argument("SOT3: Scale has to be grater than zero!");
    }
  }

  SO3Type C_;    //!< SO3 object the rotational component of SOT3
  ScaleType s_;  //!< Scale factor

  static constexpr FPType eps_ = 1e-6;  //!< Epsilon
};

using SOT3d = SOT3<double>;  //!< The SOT3 group with double precision floating point
using SOT3f = SOT3<float>;   //!< The SOT3 group with single precision floating point

}  // namespace group

#endif  // SOT3_HPP