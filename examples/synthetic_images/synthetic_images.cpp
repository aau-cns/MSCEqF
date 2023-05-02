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
  if (argc != 2)
  {
    std::cout << "Usage: ./msceqf_synthetic_images <synthetic_images_folder> without / at the end." << std::endl;
    return 1;
  }

  const std::string dataset_path = std::string(argv[1]) + "/data/";
  const std::string results_path = std::string(argv[1]) + "/results/";

  const std::vector<std::string> imu_header = {"t", "w_x", "w_y", "w_z", "a_x", "a_y", "a_z"};
  const std::vector<std::string> groundtruth_header = {"t",   "q_x", "q_y", "q_z", "q_w", "p_x",
                                                       "p_y", "p_z", "v_x", "v_y", "v_z"};
  const std::vector<std::string> cam_header = {"t", "path"};

  utils::dataParser dataset_parser(dataset_path + "trajectory_imu.csv", dataset_path + "trajectory_gt.csv",
                                   dataset_path + "images.csv", dataset_path, imu_header, groundtruth_header,
                                   cam_header);

  dataset_parser.parseAndCheck();

  std::vector<std::string> results_titles = {"t",     "q_x",   "q_y",   "q_z",   "q_w",   "p_x",   "p_y",
                                             "p_z",   "v_x",   "v_y",   "v_z",   "b_w_x", "b_w_y", "b_w_z",
                                             "b_a_x", "b_a_y", "b_a_z", "s_q_x", "s_q_y", "s_q_z", "s_q_w",
                                             "s_p_x", "s_p_y", "s_p_z", "f_x",   "f_y",   "c_x",   "c_y"};

  for (int i = 1; i <= 15; i++)
  {
    for (int j = 1; j <= 15; j++)
    {
      results_titles.emplace_back("P_" + std::to_string(i) + std::to_string(j));
    }
  }

  utils::dataWriter result_writer(results_path + "synthetic_images.csv", results_titles, ",");

  msceqf::MSCEqF sys(std::string(argv[1]) + "/config/config.yaml");

  const auto timestamps = dataset_parser.getSensorsTimestamps();
  for (const auto& timestamp : timestamps)
  {
    auto data = dataset_parser.consumeSensorReadingAt(timestamp);
    std::visit([&sys](auto&& arg) { sys.processMeasurement(arg); }, data);

    if (std::holds_alternative<msceqf::Camera>(data))
    {
      auto est = sys.stateEstimate();
      auto cov = sys.covariance().block(0, 0, 15, 15);
      result_writer << timestamp << est << cov << std::endl;
    }
  }

  return 0;
}