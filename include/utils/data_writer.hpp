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

#ifndef DATA_WRITER_HPP_
#define DATA_WRITER_HPP_

#include <filesystem>
#include <iostream>
#include <vector>

#include "msceqf/system/system.hpp"
#include "utils/logger.hpp"

namespace utils
{
class dataWriter
{
 public:
  /**
   * @brief Construct the data writer.
   * The data writer writes the estimated data in a csv file
   *
   * @param data_filename filename of csv file where the data is written to
   * @param titles titles of the columns in csv file
   * @param delimiter delimiter used in csv file
   * @note titles has to be provided according to the following order
   * @note [t, q_x, q_y, q_z, q_w, p_x, p_y, p_z, v_x, v_y, v_z, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z, s_q_x, s_q_y,
   * s_q_z, s_q_w, s_p_x, s_p_y, s_p_z, f_x, f_y, c_x, c_y]
   */
  dataWriter(const std::string& data_filename,
             const std::vector<std::string> titles,
             const std::string& delimiter = ",")
      : fs_(), filename_(data_filename), delimiter_(delimiter), dim_(titles.size())
  {
    fs_.exceptions(std::ios::failbit | std::ios::badbit);

    if (std::filesystem::exists(filename_))
    {
      std::filesystem::remove(filename_);
      Logger::info("Deleted existing file: " + filename_);
    }

    fs_.open(filename_, std::ios_base::app);
    fs_.precision(12);
    writeRow(titles);
  }

  /**
   * @brief Overload operator<< for Eigen::MatrixBase
   *
   * @tparam Derived
   * @param matrix
   * @return dataWriter&
   */
  template <typename Derived>
  dataWriter& operator<<(const Eigen::MatrixBase<Derived>& matrix)
  {
    for (int row = 0; row < matrix.rows(); ++row)
    {
      for (int col = 0; col < matrix.cols(); ++col)
      {
        fs_ << matrix(row, col);
        if (col < matrix.cols() - 1)
        {
          fs_ << delimiter_;
        }
      }
    }
    return *this;
  }

  /**
   * @brief Overload operator<< for Eigen::QuaternionBase
   *
   * @tparam Derived
   * @param quat
   * @return dataWriter&
   */
  template <typename T>
  dataWriter& operator<<(const T& val)
  {
    fs_ << val << delimiter_;
    return *this;
  }

  /**
   * @brief Overload operator<< for msceqf::SystemState
   *
   * @param state
   * @return dataWriter&
   */
  dataWriter& operator<<(const msceqf::SystemState& state)
  {
    this->operator<<(state.P().q().x());
    this->operator<<(state.P().q().y());
    this->operator<<(state.P().q().z());
    this->operator<<(state.P().q().w());
    this->operator<<(state.T().p().x());
    this->operator<<(state.T().p().y());
    this->operator<<(state.T().p().z());
    this->operator<<(state.T().v().x());
    this->operator<<(state.T().v().y());
    this->operator<<(state.T().v().z());
    this->operator<<(state.b()(0));
    this->operator<<(state.b()(1));
    this->operator<<(state.b()(2));
    this->operator<<(state.b()(3));
    this->operator<<(state.b()(4));
    this->operator<<(state.b()(5));
    this->operator<<(state.S().q().x());
    this->operator<<(state.S().q().y());
    this->operator<<(state.S().q().z());
    this->operator<<(state.S().q().w());
    this->operator<<(state.S().x().x());
    this->operator<<(state.S().x().y());
    this->operator<<(state.S().x().z());
    this->operator<<(state.k().x());
    this->operator<<(state.k().x());
    this->operator<<(state.k().z());
    this->operator<<(state.k().w());
    return *this;
  }

  /**
   * @brief Overload operator<< for std::ostream
   *
   * @param os
   * @return dataWriter&
   */
  dataWriter& operator<<(std::ostream& (*os)(std::ostream&))
  {
    os(fs_);
    return *this;
  }

 private:
  /**
   * @brief Write a row of data to the csv file
   *
   * @param row row of data
   */
  void writeRow(const std::vector<std::string>& row)
  {
    if (!fs_.is_open())
    {
      throw std::runtime_error("Error: could not open file: " + filename_ + ". Exit programm.");
    }

    if (row.size() != dim_)
    {
      throw std::runtime_error("Trying to write wrong data size.");
    }

    std::copy(row.begin(), row.end(), std::ostream_iterator<std::string>(fs_, delimiter_.c_str()));
    fs_ << "\n";
  }

  std::ofstream fs_;       //!< File stream
  std::string filename_;   //!< Name of csv file where the data is written to
  std::string delimiter_;  //!< Delimiter for csv file
  size_t dim_;             //!< Dimension of data
};
}  // namespace utils

#endif  // DATA_WRITER_HPP_