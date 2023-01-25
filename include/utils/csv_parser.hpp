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

#ifndef CSV_PARSER_HPP_
#define CSV_PARSER_HPP_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <unordered_map>
#include <vector>

#include "msceqf/system/sensor_data.hpp"
#include "utils/logger.hpp"
#include "utils/tools.hpp"

namespace msceqf
{
struct Groundtruth
{
  fp timestamp_ = -1;                      //!< Timestamp of the groundtruth
  Quaternion q_ = Quaternion::Identity();  //!< Quaternion representing rotation of IMU frame in global frame
  Vector3 p_ = Vector3::Zero();            //!< Position of IMU frame in global frame
  Vector3 v_ = Vector3::Zero();            //!< Velocity of IMU frame in global frame
  Vector3 bw_ = Vector3::Zero();           //!< Angular velocity bias
  Vector3 ba_ = Vector3::Zero();           //!< Acceleration bias

  /**
   * @brief Comparison operator with other groundtruth
   *
   */
  friend bool operator<(const Groundtruth& lhs, const Groundtruth& rhs) { return lhs.timestamp_ < rhs.timestamp_; }
};
}  // namespace msceqf

namespace utils
{
class csvParser
{
 public:
  /**
   * @brief Construct the csv parser
   *
   * @param filename
   * @param imu_header_titles
   * @param groundtruth_header_titles
   *
   * @note imu_header_titles has to be provided according to the following order
   * @note [t, ang_x, ang_y, ang_z, axx_x, axx_y, acc_z]
   * @note groundtruth_header_titles has to be provided according to the following order
   * @note [t, q_x, q_y, q_z, q_w, p_x, p_y, p_z, v_x, v_y, v_z, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z,]
   */
  csvParser(const std::string& filename,
            const std::vector<std::string>& imu_header_titles,
            const std::vector<std::string>& groundtruth_header_titles)
      : filename_(filename)
      , imu_header_titles_(imu_header_titles)
      , groundtruth_header_titles_(groundtruth_header_titles)
      , imu_data_()
      , groundtruth_data_()
  {
  }

  csvParser(std::string&& filename,
            std::vector<std::string>&& imu_header_titles,
            std::vector<std::string>&& groundtruth_header_titles)
      : filename_(filename)
      , imu_header_titles_(imu_header_titles)
      , groundtruth_header_titles_(groundtruth_header_titles)
      , imu_data_()
      , groundtruth_data_()
  {
  }

  /**
   * @brief Clear actual data, read, parse, and check the .csv file
   */
  void parseAndCheck()
  {
    std::ifstream file(filename_);

    if (!file)
    {
      throw std::runtime_error("Error opening file \"" + filename_ + "\". Exit programm.");
    }

    Logger::info("Opening and reading " + filename_ + "...");

    std::regex regex("^[+-]?((\\d*\\.\\d+)|(\\d+\\.\\d*)|(\\d+))([eE][+-]?\\d+)?|^nan$");

    std::string line;
    std::vector<std::string> header;
    std::vector<std::vector<msceqf::fp>> data;

    std::vector<int> imu_indices;
    std::vector<int> groundtruth_indices;

    int rows_cnt = 0;

    if (file.good())
    {
      std::getline(file, line);
      parseLine(line, header);

      while (std::getline(file, line))
      {
        std::vector<msceqf::fp> tmp;
        parseLine(line, tmp, regex);
        data.push_back(tmp);
        ++rows_cnt;
      }

      if (!getIndices(header, imu_header_titles_, imu_indices))
      {
        throw std::runtime_error("Required imu missing. Exit programm.");
      }

      if (!getIndices(header, groundtruth_header_titles_, groundtruth_indices))
      {
        throw std::runtime_error("Required groundtruth missing. Exit programm.");
      }

      imu_data_.clear();
      groundtruth_data_.clear();

      for (const auto& it : data)
      {
        msceqf::Imu imu;
        msceqf::Groundtruth gt;

        imu.timestamp_ = it.at(imu_indices.at(0));
        imu.ang_.x() = it.at(imu_indices.at(1));
        imu.ang_.y() = it.at(imu_indices.at(2));
        imu.ang_.z() = it.at(imu_indices.at(3));
        imu.acc_.x() = it.at(imu_indices.at(4));
        imu.acc_.y() = it.at(imu_indices.at(5));
        imu.acc_.z() = it.at(imu_indices.at(6));
        imu_data_.push_back(imu);

        gt.timestamp_ = it.at(groundtruth_indices.at(0));
        gt.q_.x() = it.at(groundtruth_indices.at(1));
        gt.q_.y() = it.at(groundtruth_indices.at(2));
        gt.q_.z() = it.at(groundtruth_indices.at(3));
        gt.q_.w() = it.at(groundtruth_indices.at(4));
        gt.p_.x() = it.at(groundtruth_indices.at(5));
        gt.p_.y() = it.at(groundtruth_indices.at(6));
        gt.p_.z() = it.at(groundtruth_indices.at(7));
        gt.v_.x() = it.at(groundtruth_indices.at(8));
        gt.v_.y() = it.at(groundtruth_indices.at(9));
        gt.v_.z() = it.at(groundtruth_indices.at(10));
        gt.bw_.x() = it.at(groundtruth_indices.at(11));
        gt.bw_.y() = it.at(groundtruth_indices.at(12));
        gt.bw_.z() = it.at(groundtruth_indices.at(13));
        gt.ba_.x() = it.at(groundtruth_indices.at(14));
        gt.ba_.y() = it.at(groundtruth_indices.at(15));
        gt.ba_.z() = it.at(groundtruth_indices.at(16));
        groundtruth_data_.push_back(gt);
      }
    }
    else
    {
      throw std::runtime_error("Corrupted file \"" + filename_ + "\". Exit programm.");
    }

    std::sort(imu_data_.begin(), imu_data_.end());
    std::sort(groundtruth_data_.begin(), groundtruth_data_.end());

    file.close();
  }

  /**
   * @brief Get a constant reference to the imu data vector
   *
   * @return const std::vector<msceqf::Imu>&
   */
  const std::vector<msceqf::Imu>& getImuData() { return imu_data_; }

  /**
   * @brief Get a constant reference to the groundtruth data vector
   *
   * @return const std::vector<msceqf::Groundtruth>&
   */
  const std::vector<msceqf::Groundtruth>& getGroundtruthData() { return groundtruth_data_; }

 private:
  /**
   * @brief Parse a single line of the .csv file
   * This method parses a single line of a .csv file, handling both header titles and numerical values.
   * This method perform regex check and it is robust to nan values.
   *
   * @param line
   * @param data
   * @param regex
   */
  template <typename T>
  void parseLine(std::string& line, std::vector<T>& data, const std::regex regex = std::regex("^[a-zA-Z0-9_]+$"))
  {
    std::stringstream ss(line);
    std::string s;

    while (std::getline(ss, s, ','))
    {
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
      if (!std::regex_match(s, regex))
      {
        throw std::runtime_error("In file: \"" + filename_ + "\": regex match fail. Exit programm.");
      }
      if constexpr (std::is_floating_point_v<T>)
      {
        if (s.find("nan") == std::string::npos)
        {
          if constexpr (std::is_same_v<T, float>)
          {
            data.push_back(std::stof(s));
          }
          else
          {
            data.push_back(std::stod(s));
          }
        }
        else
        {
          data.push_back(std::numeric_limits<msceqf::fp>::quiet_NaN());
        }
      }
      else
      {
        data.push_back(s);
      }
    }
  }

  /**
   * @brief Find association between input file header and given titles
   * This function find the indices of the columns of the input file based on its header in order to correctly
   * associate input data with the data structures allowing correctly parsing of .csv files with shuffled columns or
   * even more columns than the ones that are necessary
   *
   * @param header
   * @param titles
   * @param indices
   */
  [[nodiscard]] bool getIndices(std::vector<std::string>& header,
                                std::vector<std::string>& titles,
                                std::vector<int>& indices)
  {
    // Remove leading and trailing spaces
    std::for_each(header.begin(), header.end(), &trimString);
    std::for_each(titles.begin(), titles.end(), &trimString);

    int idx;
    for (auto& it : titles)
    {
      if (!getIndex(header, it, idx))
      {
        return false;
      }
      indices.push_back(idx);
    }
    return true;
  }

  /**
   * @brief Find the index of token within the given vector
   *
   * @tparam T
   * @param data
   * @param token
   * @param index
   * @return true if succeed, false otherwise
   */
  template <typename T>
  [[nodiscard]] bool getIndex(std::vector<T>& data, T& token, int& index)
  {
    auto it = find(data.begin(), data.end(), token);

    if (it != data.end())
    {
      index = it - data.begin();
    }
    else
    {
      return false;
    }

    return true;
  }

  std::string filename_;  //!< Filename of file containing waypoints

  std::vector<std::string> imu_header_titles_;          //!< Titles for IMU entries
  std::vector<std::string> groundtruth_header_titles_;  //!< Titles for GT entries

  std::vector<msceqf::Imu> imu_data_;                  //!< Raw IMU data read from the .csv file
  std::vector<msceqf::Groundtruth> groundtruth_data_;  //!< Raw GT data read from the .csv file
};
}  // namespace utils

#endif  // CSV_PARSER_HPP_