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

#ifndef OPTIONS_PARSER_HPP
#define OPTIONS_PARSER_HPP

#include <yaml-cpp/yaml.h>

#include <numeric>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "msceqf/options/msceqf_options.hpp"
#include "types/fptypes.hpp"
#include "utils/logger.hpp"
#include "utils/tools.hpp"

namespace msceqf
{
class OptionParser
{
 public:
  /**
   * @brief Option parser constructor
   *
   * @param filepath Parameter file
   */
  OptionParser(const std::string& filepath);

  /**
   * @brief Parse oprion and create MSCEqFOptions struct
   *
   * @return MSCEqFOptions
   */
  MSCEqFOptions parseOptions();

 private:
  /**
   * @brief Read a parameter from the YAML file and store it in a matrix or vector.
   * A vector needs to be specified as [a, b, c, ..., z] in the YAMl file.
   * A matrix needs to be specified as a a list of vectors, each representing a row of the matrix.
   *
   * @tparam Scalar
   * @tparam Rows
   * @tparam Cols
   * @param x The matrix to be filled with read values
   * @param param Parameter to be read
   * @return true if parameter correctly parsed, false otherwise
   */
  template <typename Scalar, int Rows, int Cols>
  [[nodiscard]] bool read(Eigen::Matrix<Scalar, Rows, Cols>& x, const std::string& param)
  {
    if (node_[param])
    {
      using vector = std::vector<Scalar>;
      using vectorvector = std::vector<std::vector<Scalar>>;

      int rows = Rows;
      int cols = Cols;

      if constexpr (Rows == 1 || Cols == 1)
      {
        vector vec = node_[param].as<vector>();
        if constexpr (Rows == -1)
        {
          rows = vec.size();
        }
        else if (Cols == -1)
        {
          cols = vec.size();
        }
        x = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>>(vec.data(), rows, cols);
      }
      else
      {
        vectorvector mat = node_[param].as<vectorvector>();
        vector vec = utils::flatten(mat);
        if constexpr (Rows == 1 && Cols == 1)
        {
          rows = mat.size();
          cols = vec.size() / mat.size();
        }
        x = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor>>(vec.data(), rows, cols);
      }
      if constexpr (Rows == 1)
      {
        utils::Logger::info("Parameter: [" + param + "] found. Option set to:" +
                            static_cast<std::ostringstream&>(std::ostringstream() << x).str());
      }
      else if (Cols == 1)
      {
        utils::Logger::info("Parameter: [" + param + "] found. Option set to: " +
                            static_cast<std::ostringstream&>(std::ostringstream() << x.transpose()).str());
      }
      else
      {
        utils::Logger::info("Parameter: [" + param + "] found. Option set to: \n" +
                            static_cast<std::ostringstream&>(std::ostringstream() << x).str());
      }

      return true;
    }
    utils::Logger::warn("Parameter: [" + param + "] not found");
    return false;
  }

  /**
   * @brief Read a parameter from the YAML file and store it in a quaternion.
   * The quaternion needs to be specified as [x, y, z, w] in the YAML file.
   * the function perform quaternion normalization.
   *
   * @tparam Scalar
   * @param q The quaternion to be filled with read values
   * @param parameter Parameter to be read
   * @return true if parameter correctly parsed, false otherwise
   */
  template <typename Scalar>
  [[nodiscard]] bool read(Eigen::Quaternion<Scalar>& q, const std::string& param)
  {
    if (node_[param])
    {
      using vector = std::vector<Scalar>;
      vector vec = node_[param].as<vector>();
      q = Eigen::Quaternion<Scalar>(vec).normalize();
      utils::Logger::info("Parameter: [" + param + "] found. Option set to: \n" +
                          static_cast<std::ostringstream&>(std::ostringstream() << q).str());
      return true;
    }
    utils::Logger::warn("Parameter: [" + param + "] not found");
    return false;
  }

  /**
   * @brief Read a parameter from the YAML file and store it in a variable of type T.
   *
   * @tparam T type of the variable where the parameter has to be stored in
   * @param p
   * @param param
   * @return true if parameter correctly parsed, false otherwise
   */
  template <typename T>
  [[nodiscard]] bool read(T& p, const std::string& param)
  {
    if (node_[param])
    {
      p = node_[param].as<T>();
      utils::Logger::info("Parameter: [" + param + "] found. Option set to: " +
                          static_cast<std::ostringstream&>(std::ostringstream() << p).str());
      return true;
    }
    utils::Logger::warn("Parameter: [" + param + "] not found");
    return false;
  }

  template <typename T, typename Convertible>
  void readDefault(T& p, const Convertible& def, const std::string& param)
  {
    static_assert(std::is_convertible_v<Convertible, T>);
    if (!read(p, param))
    {
      p = def;
      utils::Logger::warn("Parameter: [" + param + "] set to default value: " +
                          static_cast<std::ostringstream&>(std::ostringstream() << p).str());
    }
  }

  /**
   * @brief Parse the camera parmaeters, including extrinsics and intrinsics from the YAML file.
   * The camera parameters has to be defined according to Kalibr convention
   *
   * @param extrinsics
   * @param intrinsics
   * @param distortion_model
   * @param distortion_coefficients
   * @param resolution
   * @param timeshift_cam_imu
   * @param mask
   * @param mask_type
   */
  void parseCameraParameters(SE3& extrinsics,
                             In& intrinsics,
                             DistortionModel& distortion_model,
                             VectorX& distortion_coefficients,
                             Vector2& resolution,
                             fp& timeshift_cam_imu,
                             cv::Mat& mask,
                             MaskType& mask_type);

  /**
   * @brief Parse the given origin (initial state) and the initial time
   *
   * @param T
   * @param b
   * @param t0
   */
  void parseGivenOrigin(SE23& T, Vector6& b, fp& t0);

  /**
   * @brief Parse the image preprocessing equalization method
   *
   * @param eq
   */
  void parseEqualizationMethod(EqualizationMethod& eq);

  /**
   * @brief Parse the feature representation
   *
   * @param rep
   */
  void parseFeatureRepresentation(FeatureRepresentation& rep);

  /**
   * @brief Parse the projection method
   *
   * @param proj
   */
  void parseProjectionMethod(ProjectionMethod& proj);

  /**
   * @brief Parse the feature detector type
   *
   * @param detector
   */
  void parseDetectorType(FeatureDetector& detector);

  /**
   * @brief Parse the initial covariance
   *
   * @param D_cov Covariance of the D element of the MSCEqF state
   * @param delta_cov Covariance of the delta element of the MSCEqF state
   * @param E_cov Covariance of the E element of the MSCEqF state (Identity by default)
   * @param L_cov Covariance of the L element of the MSCEqF state (Identity by default)
   *
   */
  void parseInitialCovariance(Matrix9& D_cov, Matrix6& delta_cov, Matrix6& E_cov, Matrix4& L_cov);

  /**
   * @brief Parse the process noise
   *
   * @param w_std Standard deviation of the process noise of the angular velocity
   * @param a_std Standard deviation of the process noise of the linear acceleration
   * @param bw_std Standard deviation of the process noise of the gyroscope bias
   * @param ba_std Standard deviation of the process noise of the accelerometer bias
   */
  void parseProcessNoise(fp& w_std, fp& a_std, fp& bw_std, fp& ba_std);

  /**
   * @brief Parse the measurement noise, it transforms it in normalized pixel noise if intrinsics parameter are not
   * refined online.
   *
   * @param pix_std Standard deviation of the measurement noise of the pixel coordinates
   * @param opts State options
   */
  void parsePixStd(fp& pix_std, const StateOptions& opts);

  /**
   * @brief Parse the zero velocity update method
   *
   * @param zvu zero velocity update method
   */
  void parseZeroVelocityUpdate(ZeroVelocityUpdate& zvu);

  YAML::Node node_;       //!< YAML node
  std::string filepath_;  //!< filepath
};

}  // namespace msceqf

#endif  // OPTIONS_HPP