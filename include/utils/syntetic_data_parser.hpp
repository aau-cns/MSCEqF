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

#ifndef SYNTETIC_DATA_PARSER_HPP_
#define SYNTETIC_DATA_PARSER_HPP_

#include <utility>
#include <variant>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>

#include "sensors/sensor_data.hpp"
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
class SynteticDataParser
{
 public:
  /**
   * @brief Construct the data parser.
   * The data parser reads the groundtruth data, the imu data and the image data from a csv file
   *
   * @param imu_data_filename filename of csv file containing IMU data
   * @param groundtruth_data_filename filename of csv file containing groundtruth data
   * @param features_data_filename filename of csv file containing features data
   * @param imu_header_titles header titles of the imu data file
   * @param groundtruth_header_titles header titles of the groundtruth data file
   * @param features_header_titles  header titles of the feature data file
   *
   * @note imu_header_titles has to be provided according to the following order
   * @note [t, ang_x, ang_y, ang_z, acc_x, acc_y, acc_z]
   * @note groundtruth_header_titles has to be provided according to the following order
   * @note [t, q_x, q_y, q_z, q_w, p_x, p_y, p_z, v_x, v_y, v_z, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z]
   * @note features_header_titles has to be provided according to the following order
   * @note [t, Gf_x_1, Gf_y_1, Gf_z_1, un_1, vn_2, Gf_x_2, ...]
   */
  SynteticDataParser(const std::string& imu_data_filename,
                     const std::string& groundtruth_data_filename,
                     const std::string& features_data_filename,
                     const std::vector<std::string>& imu_header_titles,
                     const std::vector<std::string>& groundtruth_header_titles,
                     const std::vector<std::string>& features_header_titles)
      : imu_filename_(imu_data_filename)
      , groundtruth_filename_(groundtruth_data_filename)
      , features_data_filename_(features_data_filename)
      , imu_header_titles_(imu_header_titles)
      , groundtruth_header_titles_(groundtruth_header_titles)
      , features_header_titles_(features_header_titles)
      , imu_data_()
      , groundtruth_data_()
      , features_data_()
      , read_imu_(false)
      , read_gt_(false)
      , read_features_(false)
  {
    for (auto& s : imu_header_titles_)
    {
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    }
    for (auto& s : groundtruth_header_titles_)
    {
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    }
    for (auto& s : features_header_titles_)
    {
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    }
  }

  /**
   * @brief Clear actual data, read, parse, and check the .csv file
   */
  void parseAndCheck()
  {
    std::ifstream imufile(imu_filename_);
    std::ifstream gtfile(groundtruth_filename_);
    std::ifstream featfile(features_data_filename_);

    read_imu_ = imu_filename_.compare("") != 0;
    read_gt_ = groundtruth_filename_.compare("") != 0;
    read_features_ = features_data_filename_.compare("") != 0;

    if (read_gt_)
    {
      if (!gtfile)
      {
        throw std::runtime_error("Error opening GT file: \"" + groundtruth_filename_ + "\". Exit programm.");
      }

      Logger::info("Opening and reading: " + groundtruth_filename_ + "...");
      parseAndCheckGt(gtfile);
    }
    else
    {
      Logger::info("Groundtruth data file not provided. Skipping");
    }

    if (read_imu_)
    {
      if (!imufile)
      {
        throw std::runtime_error("Error opening IMU file: \"" + imu_filename_ + "\". Exit programm.");
      }

      Logger::info("Opening and reading: " + imu_filename_ + "...");
      parseAndCheckImu(imufile);
    }
    else
    {
      Logger::info("Imu data file not provided. Skipping");
    }

    if (read_features_)
    {
      if (!featfile)
      {
        throw std::runtime_error("Error opening FEATURES file: \"" + features_data_filename_ + "\". Exit programm.");
      }

      Logger::info("Opening and reading: " + features_data_filename_ + "...");
      parseAndCheckFeatures(featfile);
    }
    else
    {
      Logger::info("Features data file not provided. Skipping");
    }
  }

  void parseAndCheckGt(std::ifstream& gtfile)
  {
    std::regex regex("^[+-]?((\\d*\\.\\d+)|(\\d+\\.\\d*)|(\\d+))([eE][+-]?\\d+)?|^nan$");

    std::string line;
    std::vector<std::string> header;
    std::vector<std::vector<msceqf::fp>> data;
    std::vector<int> groundtruth_indices;

    bool have_bias = groundtruth_header_titles_.size() == 17 ? true : false;

    int rows_cnt = 0;
    if (gtfile.good())
    {
      std::getline(gtfile, line);
      parseLine(line, header);

      while (std::getline(gtfile, line))
      {
        std::vector<msceqf::fp> tmp;
        parseLine(line, tmp, regex);
        data.emplace_back(tmp);
        ++rows_cnt;
      }

      if (!getIndices(header, groundtruth_header_titles_, groundtruth_indices))
      {
        throw std::runtime_error("Required groundtruth missing. Exit programm.");
      }

      groundtruth_data_.clear();

      for (const auto& it : data)
      {
        msceqf::Groundtruth gt;

        gt.timestamp_ = it.at(groundtruth_indices.at(0)) > 10e12 ? it.at(groundtruth_indices.at(0)) / 1e9 :
                                                                   it.at(groundtruth_indices.at(0));
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
        if (have_bias)
        {
          gt.bw_.x() = it.at(groundtruth_indices.at(11));
          gt.bw_.y() = it.at(groundtruth_indices.at(12));
          gt.bw_.z() = it.at(groundtruth_indices.at(13));
          gt.ba_.x() = it.at(groundtruth_indices.at(14));
          gt.ba_.y() = it.at(groundtruth_indices.at(15));
          gt.ba_.z() = it.at(groundtruth_indices.at(16));
        }

        gt.q_.normalize();
        groundtruth_data_.emplace_back(gt);
      }
    }
    else
    {
      throw std::runtime_error("Corrupted file. Exit programm.");
    }

    std::sort(groundtruth_data_.begin(), groundtruth_data_.end());

    gtfile.close();
  }

  void parseAndCheckImu(std::ifstream& imufile)
  {
    std::regex regex("^[+-]?((\\d*\\.\\d+)|(\\d+\\.\\d*)|(\\d+))([eE][+-]?\\d+)?|^nan$");

    std::string line;
    std::vector<std::string> header;
    std::vector<std::vector<msceqf::fp>> data;
    std::vector<int> imu_indices;

    int rows_cnt = 0;
    if (imufile.good())
    {
      std::getline(imufile, line);
      parseLine(line, header);

      while (std::getline(imufile, line))
      {
        std::vector<msceqf::fp> tmp;
        parseLine(line, tmp, regex);
        data.emplace_back(tmp);
        ++rows_cnt;
      }

      if (!getIndices(header, imu_header_titles_, imu_indices))
      {
        throw std::runtime_error("Required imu missing. Exit programm.");
      }

      imu_data_.clear();

      for (const auto& it : data)
      {
        msceqf::Imu imu;

        imu.timestamp_ = it.at(imu_indices.at(0)) > 10e12 ? it.at(imu_indices.at(0)) / 1e9 : it.at(imu_indices.at(0));
        imu.ang_.x() = it.at(imu_indices.at(1));
        imu.ang_.y() = it.at(imu_indices.at(2));
        imu.ang_.z() = it.at(imu_indices.at(3));
        imu.acc_.x() = it.at(imu_indices.at(4));
        imu.acc_.y() = it.at(imu_indices.at(5));
        imu.acc_.z() = it.at(imu_indices.at(6));

        imu_data_.emplace_back(imu);
      }
    }
    else
    {
      throw std::runtime_error("Corrupted file. Exit programm.");
    }

    std::sort(imu_data_.begin(), imu_data_.end());

    imufile.close();
  }

  void parseAndCheckFeatures(std::ifstream& featfile)
  {
    std::regex regex("^[+-]?((\\d*\\.\\d+)|(\\d+\\.\\d*)|(\\d+))([eE][+-]?\\d+)?|^nan$");

    std::string line;
    std::vector<std::string> header;
    std::vector<std::vector<msceqf::fp>> data;
    std::vector<int> feats_indices;

    int rows_cnt = 0;
    if (featfile.good())
    {
      std::getline(featfile, line);
      parseLine(line, header);

      while (std::getline(featfile, line))
      {
        std::vector<msceqf::fp> tmp;
        parseLine(line, tmp, regex);
        data.emplace_back(tmp);
        ++rows_cnt;
      }

      if (!getIndices(header, features_header_titles_, feats_indices))
      {
        throw std::runtime_error("Required imu missing. Exit programm.");
      }

      int num_features = (feats_indices.size() - 1) / 5;

      features_data_.clear();

      for (const auto& it : data)
      {
        msceqf::TriangulatedFeatures feat;

        int cnt = 1;
        bool found = false;

        for (int i = 1; i < (5 * num_features); i += 5)
        {
          if (!std::isnan(it.at(feats_indices.at(i))))
          {
            found = true;

            feat.timestamp_ =
                it.at(feats_indices.at(0)) > 10e12 ? it.at(feats_indices.at(0)) / 1e9 : it.at(feats_indices.at(0));
            feat.points_.emplace_back(it.at(feats_indices.at(i)), it.at(feats_indices.at(i + 1)),
                                      it.at(feats_indices.at(i + 2)));
            feat.features_.distorted_uvs_.emplace_back(-1, -1);
            feat.features_.uvs_.emplace_back(-1, -1);
            feat.features_.normalized_uvs_.emplace_back(it.at(feats_indices.at(i + 3)), it.at(feats_indices.at(i + 4)));
            feat.features_.ids_.emplace_back(cnt);
          }
          ++cnt;
        }

        if (found)
        {
          features_data_.emplace_back(feat);
        }
      }
    }
    else
    {
      throw std::runtime_error("Corrupted file. Exit programm.");
    }

    std::sort(features_data_.begin(), features_data_.end());

    featfile.close();
  }

  /**
   * @brief Get a constant reference to the imu data vector
   *
   * @return const std::vector<msceqf::Imu>&
   */
  const std::vector<msceqf::Imu>& getImuData() const { return imu_data_; }

  /**
   * @brief Get a constant reference to the groundtruth data vector
   *
   * @return const std::vector<msceqf::Groundtruth>&
   */
  const std::vector<msceqf::Groundtruth>& getGroundtruthData() const { return groundtruth_data_; }

  /**
   * @brief Get a constant reference to the features data vector
   *
   * @return const std::vector<msceqf::TriangulatedFeatures>&
   */
  const std::vector<msceqf::TriangulatedFeatures>& getFeaturesData() const { return features_data_; }

  /**
   * @brief Get a vector containing the timestamps of the sensors measurements (imu and features).
   * The returned vector is sorted in ascending order
   *
   * @return const std::vector<msceqf::fp>
   */
  const std::vector<msceqf::fp> getSensorsTimestamps() const
  {
    std::vector<msceqf::fp> timestamps;
    for (const auto& imu : imu_data_)
    {
      timestamps.emplace_back(imu.timestamp_);
    }
    for (const auto& feat : features_data_)
    {
      timestamps.emplace_back(feat.timestamp_);
    }
    std::sort(timestamps.begin(), timestamps.end());
    return timestamps;
  }

  /**
   * @brief Get the sensor (imu or features) reading at a given timestamp and remove it from the internal data
   * structures.
   *
   * @param timestamp
   * @return const std::variant<msceqf::Imu, msceqf::TriangulatedFeatures>
   */
  const std::variant<msceqf::Imu, msceqf::TriangulatedFeatures> consumeSensorReadingAt(const msceqf::fp& timestamp)
  {
    auto imu_it =
        std::find_if(imu_data_.begin(), imu_data_.end(), [&](const auto& imu) { return imu.timestamp_ == timestamp; });
    auto feat_it = std::find_if(features_data_.begin(), features_data_.end(),
                                [&](const auto& feat) { return feat.timestamp_ == timestamp; });

    std::variant<msceqf::Imu, msceqf::TriangulatedFeatures> data;

    if (imu_it != imu_data_.end())
    {
      data = *imu_it;
      imu_data_.erase(imu_it);
      return data;
    }

    if (feat_it != features_data_.end())
    {
      data = *feat_it;
      features_data_.erase(feat_it);
      return data;
    }

    throw std::runtime_error("No sensor reading found at timestamp " + std::to_string(timestamp));
  }

  /**
   * @brief Get Groundtruth data that at a given timestamp and remove it from the internal data structure.
   *
   * @param timestamp
   * @return msceqf::Groundtruth
   */
  const msceqf::Groundtruth consumeGroundtruthAt(const msceqf::fp& timestamp)
  {
    auto gt_it = std::find_if(groundtruth_data_.begin(), groundtruth_data_.end(),
                              [&](const auto& gt) { return gt.timestamp_ == timestamp; });

    if (gt_it != groundtruth_data_.end())
    {
      auto data = *gt_it;
      groundtruth_data_.erase(gt_it);
      return data;
    }

    throw std::runtime_error("No groundtruth data found at timestamp " + std::to_string(timestamp));
  }

 private:
  /**
   * @brief Parse a single line of the .csv file
   * This method parses a single line of a .csv file.
   * This method perform regex check based on the given regex. By default every string is allowed.
   *
   * @param line
   * @param data
   * @param regex
   */
  template <typename T>
  void parseLine(std::string& line, std::vector<T>& data, const std::regex regex = std::regex("^.*$"))
  {
    std::stringstream ss(line);
    std::string s;

    while (std::getline(ss, s, ','))
    {
      if (s.back() == '\r')
      {
        s.pop_back();
      }
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
      if (!std::regex_match(s, regex))
      {
        throw std::runtime_error("Regex match fail. Exit programm.");
      }
      if constexpr (std::is_floating_point_v<T>)
      {
        if (s.find("nan") == std::string::npos)
        {
          if constexpr (std::is_same_v<T, float>)
          {
            data.emplace_back(std::stof(s));
          }
          else
          {
            data.emplace_back(std::stod(s));
          }
        }
        else
        {
          data.emplace_back(std::numeric_limits<msceqf::fp>::quiet_NaN());
        }
      }
      else
      {
        data.emplace_back(s);
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
      indices.emplace_back(idx);
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

  std::string imu_filename_;            //!< Name of csv file containing imu data
  std::string groundtruth_filename_;    //!< Name of csv file containing groundtruth data
  std::string features_data_filename_;  //!< Name of csv file containing features data

  std::vector<std::string> imu_header_titles_;          //!< Titles for imu entries
  std::vector<std::string> groundtruth_header_titles_;  //!< Titles for GT entries
  std::vector<std::string> features_header_titles_;     //!< Titles for features entries

  std::vector<msceqf::Imu> imu_data_;                        //!< Raw imu data read from the .csv file
  std::vector<msceqf::Groundtruth> groundtruth_data_;        //!< Raw groundtruth data read from the .csv file
  std::vector<msceqf::TriangulatedFeatures> features_data_;  //!< features data read from the .csv file

  bool read_imu_;       //!< Flag to read IMU data
  bool read_gt_;        //!< Flag to read GT data
  bool read_features_;  //!< Flag to read features data
};
}  // namespace utils

#endif  // SYNTETIC_DATA_PARSER_HPP_