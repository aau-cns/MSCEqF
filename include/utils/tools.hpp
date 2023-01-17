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

#ifndef UTILS_HPP
#define UTILS_HPP

#include <array>
#include <iterator>
#include <random>
#include <vector>

namespace utils
{
/**
 * @brief Stream a std::vector
 *
 * @tparam T type of data to be streamed
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& v)
{
  // Check container is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of container into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

/**
 * @brief Stream a std::array
 *
 * @tparam T type of data to be streamed
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& stream, const std::array<T, N>& v)
{
  // Check container is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of container into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

/**
 * @brief Very simple Eigen compatible central difference numerical differentiation function.
 *
 * @tparam FPType floating point type
 * @param x linearization point
 * @param f function
 * @param h delta
 * @return Eigen::Matrix<FPType, Eigen::Dynamic, 1>
 */
template <typename FPType>
Eigen::Matrix<FPType, Eigen::Dynamic, 1> diff(
    const Eigen::Matrix<FPType, Eigen::Dynamic, 1>& x,
    const std::function<double(const Eigen::Matrix<FPType, Eigen::Dynamic, 1>&)>& f,
    double h = 1e-6)
{
  int n = x.size();
  Eigen::Matrix<FPType, Eigen::Dynamic, 1> grad(n);
  for (int i = 0; i < n; i++)
  {
    Eigen::Matrix<FPType, Eigen::Dynamic, 1> x_plus = x, x_minus = x;
    x_plus(i) += h;
    x_minus(i) -= h;
    grad(i) = (f(x_plus) - f(x_minus)) / (2 * h);
  }
  return grad;
}

/**
 * @brief Generate random numbers
 *
 * @tparam Numeric type of random number generated
 * @tparam Generator random number generator
 * @param from lower bound
 * @param to upper bound
 * @return Numeric
 *
 * @note Modified from:
 * https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
 */
template <typename Numeric, typename Generator = std::mt19937>
Numeric random(Numeric from, Numeric to)
{
  thread_local static Generator gen(std::random_device{}());
  using dist_type = typename std::conditional<std::is_integral<Numeric>::value, std::uniform_int_distribution<Numeric>,
                                              std::uniform_real_distribution<Numeric> >::type;
  thread_local static dist_type dist;
  return dist(gen, typename dist_type::param_type{from, to});
}

}  // namespace utils

#endif  // UTILS_HPP
