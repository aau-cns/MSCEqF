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
#include <vector>

namespace utils
{
/**
 * @brief Helper function to unpack a vector into function arguments and call the given function.
 * unpackCaller<N> u; u(f, v, pre_args, post_args); calls f(pre_args..., v[0], v[1], ..., v[N], post_args...)
 *
 * @tparam N
 *
 * @note Modified from:
 * https://stackoverflow.com/questions/11044504/any-solution-to-unpack-a-vector-to-function-arguments-in-c/11044592#11044592
 */
template <size_t N>
class unpackCaller
{
 public:
  template <typename F, typename T, typename... PreArgs, typename... PostArgs>
  auto operator()(F& f, std::vector<T>& other_args, PreArgs&&... pre_args, PostArgs&&... post_args)
  {
    assert(other_args.size() == N);
    return call(f, other_args, std::make_index_sequence<N>{}, pre_args..., post_args...);
  }

 private:
  template <typename F, typename T, size_t... I, typename... PreArgs, typename... PostArgs>
  auto call(F& f, std::vector<T>& other_args, std::index_sequence<I...>, PreArgs&&... pre_args, PostArgs&&... post_args)
  {
    return f(pre_args..., other_args[I]..., post_args...);
  }
};

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

}  // namespace utils

#endif  // UTILS_HPP
