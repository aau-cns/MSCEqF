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

#ifndef UTILS_HPP
#define UTILS_HPP

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
}  // namespace utils

#endif  // UTILS_HPP
