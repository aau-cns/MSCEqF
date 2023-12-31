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

#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <queue>
#include <random>
#include <type_traits>
#include <Eigen/Dense>

namespace utils
{
/**
 * @brief This calss define a map that keeps the insertion order.
 *
 * @tparam Key
 * @tparam Value
 */
template <typename Key, typename Value>
class InsertionOrderedMap
{
 public:
  /**
   * @brief Insert a key-value pair into the map if the key does not exists.
   *
   * @param key Key
   * @param value Value
   */
  void insert(const Key& key, const Value& value)
  {
    if (map_.find(key) == map_.end())
    {
      map_[key] = vector_.size();
      vector_.emplace_back(std::make_pair(key, value));
    }
  }

  /**
   * @brief Return the value associated with the key.
   *
   * @param key
   * @return Value
   */
  const Value& at(const Key& key) const
  {
    auto index = map_.at(key);
    return vector_.at(index).second;
  }

  /**
   * @brief Return the value associated with the key.
   *
   * @param key
   * @return Value
   */
  Value& at(const Key& key)
  {
    auto index = map_.at(key);
    return vector_.at(index).second;
  }

  /**
   * @brief Return a vector containing the keys
   *
   * @return Vector of keys
   */
  const std::vector<Key> keys() const
  {
    std::vector<Key> result;
    for (const auto& pair : vector_)
    {
      result.push_back(pair.first);
    }
    return result;
  }

  /**
   * @brief Return a vector containing the values
   *
   * @return Vector of values
   */
  const std::vector<Value> values() const
  {
    std::vector<Value> result;
    for (const auto& pair : vector_)
    {
      result.push_back(pair.second);
    }
    return result;
  }

  /**
   * @brief Clear the map and the vector
   *
   */
  void clear()
  {
    map_.clear();
    vector_.clear();
  }

 private:
  std::unordered_map<Key, size_t> map_;
  std::vector<std::pair<Key, Value>> vector_;
};

/**
 * @brief Very simple Eigen compatible central difference numerical differentiation function.
 *
 * @tparam FPType floating point type
 * @param x Linearization point
 * @param f Function
 * @param h Delta
 * @return Jacobian matrix
 */
template <typename FPType>
static Eigen::Matrix<FPType, Eigen::Dynamic, 1> diff(
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
 * @param from Lower bound
 * @param to Upper bound
 * @return Numeric
 *
 * @note Modified from:
 * https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
 */
template <typename Numeric, typename Generator = std::mt19937>
static Numeric random(Numeric from, Numeric to)
{
  thread_local static Generator gen(std::random_device{}());
  using dist_type = typename std::conditional<std::is_integral<Numeric>::value, std::uniform_int_distribution<Numeric>,
                                              std::uniform_real_distribution<Numeric>>::type;
  thread_local static dist_type dist;
  return dist(gen, typename dist_type::param_type{from, to});
}

/**
 * @brief Trim a string, remove leading and trailing spaces
 *
 * @param s String to be trimmed
 */
static inline void trimString(std::string& s)
{
  s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), [](unsigned char ch) { return std::isspace(ch); }));
  s.erase(std::find_if_not(s.rbegin(), s.rend(), [](unsigned char ch) { return std::isspace(ch); }).base(), s.end());
}

/**
 * @brief Flatten a vector of vectors
 *
 * @tparam Type of data in vector
 * @param vector_of_vectors Vector of vectors
 * @return Flatten vector
 */
template <typename T>
static std::vector<T> flatten(const std::vector<std::vector<T>>& vector_of_vectors)
{
  std::vector<T> flat;
  size_t total_size = std::accumulate(vector_of_vectors.begin(), vector_of_vectors.end(), 0,
                                      [](size_t size, const std::vector<T>& vec) { return size + vec.size(); });
  flat.reserve(total_size);
  for (const auto& vec : vector_of_vectors)
  {
    flat.insert(flat.end(), vec.begin(), vec.end());
  }
  return flat;
}

// /**
//  * @brief Flatten a vector of vectors and insert in given vector (append if the given vector in non empty)
//  *
//  * @tparam Type of data in vector
//  * @param vector_of_vectors Vector of vectors
//  * @return Flatten vector
//  *
//  * @note The vector of vectors is moved into flat so it becames unusable
//  */
// template <typename T>
// static void flattenInto(const std::vector<std::vector<T>>& vector_of_vectors, std::vector<T>& flat)
// {
//   size_t total_size =
//       flat.size() + std::accumulate(vector_of_vectors.begin(), vector_of_vectors.end(), 0,
//                                     [](size_t size, const std::vector<T>& vec) { return size + vec.size(); });
//   flat.reserve(total_size);
//   for (const auto& vec : vector_of_vectors)
//   {
//     flat.insert(flat.end(), std::make_move_iterator(vec.begin()), std::make_move_iterator(vec.end()));
//   }
// }

/**
 * @brief Perform 2^n with an integer n
 *
 * @param n Power value
 * @return Result
 */
static inline int pow2(const int& n) { return static_cast<int>(std::ldexp(1.0f, n)); }

}  // namespace utils

/**
 * @brief Stream a std::vector
 *
 * @tparam T type of data to be streamed
 * @param stream Stream
 * @param x Data to be streamed
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
 * @param stream Stream
 * @param x Data to be streamed
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
 * @brief Stream a std::deque
 *
 * @tparam T type of data to be streamed
 * @param stream Stream
 * @param x Data to be streamed
 */
template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::deque<T>& v)
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
 * @brief Stream an enum
 *
 * @tparam T type of data to be streamed (enum)
 * @param stream Stream
 * @param x Data to be streamed
 */
template <typename T, typename std::enable_if<std::is_enum<T>::value, T>::type* = nullptr>
std::ostream& operator<<(std::ostream& stream, const T& e)
{
  return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

#endif  // TOOLS_HPP
