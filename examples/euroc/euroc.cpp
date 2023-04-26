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

#include "msceqf/msceqf.hpp"
#include "utils/data_parser.hpp"
#include "utils/data_writer.hpp"

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    std::cout << "Usage: ./msceqf_euroc <dataset_name> <dataset_folder> <euroc_folder> Each without / at the end."
              << std::endl;
    return 1;
  }

  const std::string dataset_name = argv[1];
  const std::string dataset_path = std::string(argv[2]) + "/" + dataset_name;
  const std::string results_path = std::string(argv[3]) + "/results/" + dataset_name + ".csv";
  const std::string imu_path = dataset_path + "/mav0/imu0/data.csv";
  const std::string cam_path = dataset_path + "/mav0/cam0/data.csv";
  const std::string cam_image_path = dataset_path + "/mav0/cam0/data/";
  const std::string groundtruth_path = dataset_path + "/mav0/state_groundtruth_estimate0/data.csv";

  const std::vector<std::string> imu_header = {"#timestamp [ns]",     "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                                               "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]",   "a_RS_S_y [m s^-2]",
                                               "a_RS_S_z [m s^-2]"};

  const std::vector<std::string> groundtruth_header = {"#timestamp",
                                                       "q_RS_x []",
                                                       "q_RS_y []",
                                                       "q_RS_z []",
                                                       "q_RS_w []",
                                                       "p_RS_R_x [m]",
                                                       "p_RS_R_y [m]",
                                                       "p_RS_R_z [m]",
                                                       "v_RS_R_x [m s^-1]",
                                                       "v_RS_R_y [m s^-1]",
                                                       "v_RS_R_z [m s^-1]",
                                                       "b_w_RS_S_x [rad s^-1]",
                                                       "b_w_RS_S_y [rad s^-1]",
                                                       "b_w_RS_S_z [rad s^-1]",
                                                       "b_a_RS_S_x [m s^-2]",
                                                       "b_a_RS_S_y [m s^-2]",
                                                       "b_a_RS_S_z [m s^-2]"};

  const std::vector<std::string> cam_header = {"#timestamp [ns]", "filename"};

  std::vector<std::string> results_titles = {"t",     "q_x",   "q_y",   "q_z",   "q_w",   "p_x",   "p_y",
                                             "p_z",   "v_x",   "v_y",   "v_z",   "b_w_x", "b_w_y", "b_w_z",
                                             "b_a_x", "b_a_y", "b_a_z", "s_q_x", "s_q_y", "s_q_z", "s_q_w",
                                             "s_p_x", "s_p_y", "s_p_z", "f_x",   "f_y",   "c_x",   "c_y"};

  for (int i = 1; i <= 9; i++)
  {
    for (int j = 1; j <= 9; j++)
    {
      results_titles.emplace_back("P_" + std::to_string(i) + std::to_string(j));
    }
  }

  utils::dataParser dataset_parser(imu_path, groundtruth_path, cam_path, cam_image_path, imu_header, groundtruth_header,
                                   cam_header);

  dataset_parser.parseAndCheck();

  utils::dataWriter result_writer(results_path, results_titles, ",");

  msceqf::MSCEqF sys(std::string(argv[3]) + "/config/config.yaml");

  const auto timestamps = dataset_parser.getSensorsTimestamps();
  for (const auto& timestamp : timestamps)
  {
    auto data = dataset_parser.consumeSensorReadingAt(timestamp);
    if (std::holds_alternative<msceqf::Imu>(data))
    {
      auto imu = std::get<msceqf::Imu>(data);
      sys.processMeasurement(imu);
    }
    else if (std::holds_alternative<msceqf::Camera>(data))
    {
      sys.processMeasurement(std::get<msceqf::Camera>(data));
      auto est = sys.stateEstimate();
      auto cov = sys.covariance().block(0, 0, 9, 9);
      result_writer << timestamp << est << cov << std::endl;
      sys.visualizeImageWithTracks(std::get<msceqf::Camera>(data));
    }
  }

  return 0;
}