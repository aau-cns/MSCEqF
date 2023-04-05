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

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <iostream>
#include <memory>

namespace utils
{

template <class S, class C, typename = void>
struct is_streamable : ::std::false_type
{
};

template <class S, class C>
struct is_streamable<S, C, decltype(void(std::declval<S&>() << std::declval<C const&>()))> : ::std::true_type
{
};

enum class LoggerLevel
{
  FULL,
  INFO,
  WARN,
  ERR,
  INACTIVE,
};

class Logger
{
 public:
  /**
   * @brief Get the logger level (see LoggerLevel)
   *
   * @return const LoggerLevel&
   */
  static const LoggerLevel& getlevel() { return level_; }

  /**
   * @brief Set the logger level (see LoggerLevel)
   *
   * @param level LoggerLevel
   */
  static void setLevel(const LoggerLevel& level) { level_ = level; }

  /**
   * @brief Format a info message and log it in white
   *
   * @tparam T Type of the message
   * @param msg message
   */
  template <typename T>
  static void info(const T& msg)
  {
    static_assert(is_streamable<std::ostream, T>::value);
    if (level_ == LoggerLevel::INFO || level_ == LoggerLevel::FULL)
    {
      std::cout << "[ INFO]: " << msg << '.' << std::endl;
    }
  }

  /**
   * @brief Format a error message and log it in red
   *
   * @tparam T Type of the message
   * @param msg message
   */
  template <typename T>
  static void err(const T& msg)
  {
    static_assert(is_streamable<std::ostream, T>::value);
    if (level_ == LoggerLevel::INFO || level_ == LoggerLevel::WARN || level_ == LoggerLevel::ERR ||
        level_ == LoggerLevel::FULL)
    {
      std::cout << "\033[31m[ ERROR]: " << msg << ".\033[0m" << std::endl;
    }
  }

  /**
   * @brief Format a warn message and log it in yellow
   *
   * @tparam T Type of the message
   * @param msg message
   */
  template <typename T>
  static void warn(const T& msg)
  {
    static_assert(is_streamable<std::ostream, T>::value);
    if (level_ == LoggerLevel::INFO || level_ == LoggerLevel::WARN || level_ == LoggerLevel::FULL)
    {
      std::cout << "\033[33m[ WARNING]: " << msg << ".\033[0m" << std::endl;
    }
  }

  /**
   * @brief Format a debug message and log it in blue
   *
   * @tparam T Type of the message
   * @param msg message
   */
  template <typename T>
  static void debug(const T& msg)
  {
    static_assert(is_streamable<std::ostream, T>::value);
    if (level_ == LoggerLevel::FULL)
    {
      std::cout << "\033[34m[ DEBUG]: " << msg << ".\033[0m" << std::endl;
    }
  }

 private:
  static inline LoggerLevel level_ = LoggerLevel::INFO;  //!< Logger level (INFO by default)
};

}  // namespace utils

#endif  // LOGGER_HPP_