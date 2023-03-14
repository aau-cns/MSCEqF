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

#ifndef DATA_PARSER_HPP_
#define DATA_PARSER_HPP_

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
class dataParser
{
 public:
  /**
   * @brief Construct the data parser.
   * The data parser reads the groundtruth data, the imu data and the image data from a csv file
   *
   * @param imu_data_filename filename of csv file containing IMU data
   * @param groundtruth_data_filename filename of csv file containing groundtruth data
   * @param image_data_filename filename of csv file containing image data (timestamp and image filename)
   * @param image_data_folder_ folder containing the images
   * @param imu_header_titles header titles of the imu data file
   * @param groundtruth_header_titles header titles of the groundtruth data file
   * @param image_header_titles  header titles of the image data file
   *
   * @note imu_header_titles has to be provided according to the following order
   * @note [t, ang_x, ang_y, ang_z, acc_x, acc_y, acc_z]
   * @note groundtruth_header_titles has to be provided according to the following order
   * @note [t, q_x, q_y, q_z, q_w, p_x, p_y, p_z, v_x, v_y, v_z, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z]
   * @note image_header_titles has to be provided according to the following order
   * @note [t, img_filename]
   */
  dataParser(const std::string& imu_data_filename,
             const std::string& groundtruth_data_filename,
             const std::string& image_data_filename,
             const std::string& image_data_folder_,
             const std::string& features_data_filename,
             const std::vector<std::string>& imu_header_titles,
             const std::vector<std::string>& groundtruth_header_titles,
             const std::vector<std::string>& image_header_titles,
             const std::vector<std::string>& features_header_titles)
      : imu_filename_(imu_data_filename)
      , groundtruth_filename_(groundtruth_data_filename)
      , image_data_filename_(image_data_filename)
      , image_data_folder_(image_data_folder_)
      , features_data_filename_(features_data_filename)
      , imu_header_titles_(imu_header_titles)
      , groundtruth_header_titles_(groundtruth_header_titles)
      , image_header_titles_(image_header_titles)
      , features_header_titles_(features_header_titles)
      , imu_data_()
      , groundtruth_data_()
      , image_data_()
      , features_data_()
      , read_imu_(false)
      , read_gt_(false)
      , read_images_(false)
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
    for (auto& s : image_header_titles_)
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
    std::ifstream imgfile(image_data_filename_);
    std::ifstream featfile(features_data_filename_);

    read_imu_ = imu_filename_.compare("") != 0;
    read_gt_ = groundtruth_filename_.compare("") != 0;
    read_images_ = image_data_filename_.compare("") != 0;
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

    if (read_images_)
    {
      if (!(imgfile && std::filesystem::is_directory(image_data_folder_)))
      {
        throw std::runtime_error("Error opening IMAGE file: \"" + image_data_filename_ + "\". Exit programm.");
      }

      Logger::info("Opening and reading: " + image_data_filename_ + "...");
      parseAndCheckImages(imgfile);
    }
    else
    {
      Logger::info("Image data file not provided. Skipping");
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
        gt.q_.normalize();

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

  void parseAndCheckImages(std::ifstream& imgfile)
  {
    std::string line;
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> data;
    std::vector<int> image_indices;

    int rows_cnt = 0;
    if (imgfile.good())
    {
      std::getline(imgfile, line);
      parseLine(line, header);

      while (std::getline(imgfile, line))
      {
        std::vector<std::string> tmp;
        parseLine(line, tmp);
        data.emplace_back(tmp);
        ++rows_cnt;
      }

      if (!getIndices(header, image_header_titles_, image_indices))
      {
        throw std::runtime_error("Required groundtruth missing. Exit programm.");
      }

      image_data_.clear();

      for (const auto& it : data)
      {
        msceqf::Camera cam;

        cam.timestamp_ = std::stod(it.at(image_indices.at(0))) > 10e12 ? std::stod(it.at(image_indices.at(0))) / 1e9 :
                                                                         std::stod(it.at(image_indices.at(0)));

        cam.image_ = cv::imread(image_data_folder_ + it.at(image_indices.at(1)));

        cam.mask_ = cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);

        image_data_.emplace_back(cam);
      }
    }
    else
    {
      throw std::runtime_error("Corrupted file. Exit programm.");
    }

    std::sort(image_data_.begin(), image_data_.end());

    imgfile.close();
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

      int cnt = 1;
      for (const auto& it : data)
      {
        if (std::isnan(it.at(feats_indices.at(1))))
        {
          ++cnt;
          continue;
        }

        msceqf::TriangulatedFeatures feat;

        feat.timestamp_ =
            it.at(feats_indices.at(0)) > 10e12 ? it.at(feats_indices.at(0)) / 1e9 : it.at(feats_indices.at(0));

        for (int i = 1; i < (5 * num_features); i += 5)
        {
          feat.points_.emplace_back(it.at(feats_indices.at(i)), it.at(feats_indices.at(i + 1)),
                                    it.at(feats_indices.at(i + 2)));

          feat.features_.uvs_.emplace_back(-1, -1);
          feat.features_.normalized_uvs_.emplace_back(it.at(feats_indices.at(i + 3)), it.at(feats_indices.at(i + 4)));
          feat.features_.ids_.emplace_back(cnt++);
        }

        features_data_.emplace_back(feat);
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
   * @brief Get a constant reference to the image data vector
   *
   * @return const std::vector<msceqf::Camera>&
   */
  const std::vector<msceqf::Camera>& getImageData() const { return image_data_; }

  /**
   * @brief Get a vector containing the timestamps of the sensors measurements (imu and camera).
   * The returned vector is sorted in ascending order and without duplicates.
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
    for (const auto& cam : image_data_)
    {
      timestamps.emplace_back(cam.timestamp_);
    }
    for (const auto& feat : features_data_)
    {
      timestamps.emplace_back(feat.timestamp_);
    }
    std::sort(timestamps.begin(), timestamps.end());
    timestamps.erase(std::unique(timestamps.begin(), timestamps.end()), timestamps.end());
    return timestamps;
  }

  /**
   * @brief Get the sensor (imu or camera) reading at a given timestamp
   *
   * @param timestamp
   * @return const std::variant<msceqf::Imu, msceqf::Camera>
   */
  const std::variant<msceqf::Imu, msceqf::Camera> consumeSensorReadingAt(const msceqf::fp& timestamp)
  {
    auto imu_it =
        std::find_if(imu_data_.begin(), imu_data_.end(), [&](const auto& imu) { return imu.timestamp_ == timestamp; });
    auto cam_it = std::find_if(image_data_.begin(), image_data_.end(),
                               [&](const auto& cam) { return cam.timestamp_ == timestamp; });

    std::variant<msceqf::Imu, msceqf::Camera> data;

    if (imu_it != imu_data_.end())
    {
      data = *imu_it;
      imu_data_.erase(imu_it);
      return data;
    }

    if (cam_it != image_data_.end())
    {
      data = *cam_it;
      image_data_.erase(cam_it);
      return data;
    }

    throw std::runtime_error("No sensor reading found at timestamp " + std::to_string(timestamp));
  }

  /**
   * @brief Get Groundtruth data that is closer to a given timestamp
   *
   * @param timestamp
   * @return msceqf::Groundtruth
   */
  const msceqf::Groundtruth getCloserGroundtruthAt(const msceqf::fp& timestamp) const
  {
    auto gt = std::find_if(groundtruth_data_.begin(), groundtruth_data_.end(),
                           [&](const auto& gt) { return gt.timestamp_ > timestamp; });

    if (gt == groundtruth_data_.end())
    {
      throw std::runtime_error("No groundtruth data found at timestamp " + std::to_string(timestamp));
    }

    if (gt != groundtruth_data_.begin() &&
        (std::abs(gt->timestamp_ - timestamp) > std::abs((gt - 1)->timestamp_ - timestamp)))
    {
      return *(gt - 1);
    }

    return *gt;
  }

  /**
   * @brief Get the feature data that is closer to a given timestamp
   *
   * @param timestamp
   * @return msceqf::Feature
   */
  const msceqf::TriangulatedFeatures getCloserFeatureDataAt(const msceqf::fp& timestamp) const
  {
    auto feat = std::find_if(features_data_.begin(), features_data_.end(),
                             [&](const auto& feat) { return feat.timestamp_ > timestamp; });

    if (feat == features_data_.end())
    {
      throw std::runtime_error("No feature data found at timestamp " + std::to_string(timestamp));
    }

    if (feat != features_data_.begin() &&
        (std::abs(feat->timestamp_ - timestamp) > std::abs((feat - 1)->timestamp_ - timestamp)))
    {
      return *(feat - 1);
    }

    return *feat;
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

  std::string imu_filename_;          //!< Name of csv file containing IMU data
  std::string groundtruth_filename_;  //!< Name of csv file containing GT data
  std::string image_data_filename_;   //!< Name of csv file containing IMAGE data
  std::string image_data_folder_;     //!< Folder containing IMAGE data

  std::string features_data_filename_;

  std::vector<std::string> imu_header_titles_;          //!< Titles for IMU entries
  std::vector<std::string> groundtruth_header_titles_;  //!< Titles for GT entries
  std::vector<std::string> image_header_titles_;        //!< Titles for IMAGE entries

  std::vector<std::string> features_header_titles_;

  std::vector<msceqf::Imu> imu_data_;                  //!< Raw IMU data read from the .csv file
  std::vector<msceqf::Groundtruth> groundtruth_data_;  //!< Raw GT data read from the .csv file
  std::vector<msceqf::Camera> image_data_;             //!< Raw IMAGE data read from the .csv file

  std::vector<msceqf::TriangulatedFeatures> features_data_;

  bool read_imu_;     //!< Flag to read IMU data
  bool read_gt_;      //!< Flag to read GT data
  bool read_images_;  //!< Flag to read IMAGE data

  bool read_features_;
};
}  // namespace utils

#endif  // DATA_PARSER_HPP_