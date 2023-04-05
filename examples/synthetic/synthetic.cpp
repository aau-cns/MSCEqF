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
#include "utils/syntetic_data_parser.hpp"
#include "utils/data_writer.hpp"

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: ./msceqf_synthetic <synthetic_folder> without / at the end." << std::endl;
    return 1;
  }

  const std::string dataset_path = std::string(argv[1]) + "/data/";
  const std::string results_path = std::string(argv[1]) + "/results/";

  const std::vector<std::string> imu_header = {"t", "w_x", "w_y", "w_z", "a_x", "a_y", "a_z"};
  const std::vector<std::string> groundtruth_header = {"t",   "q_x", "q_y", "q_z", "q_w", "p_x",
                                                       "p_y", "p_z", "v_x", "v_y", "v_z"};
  std::vector<std::string> feats_header = {"t"};
  for (int i = 1; i <= 500; ++i)
  {
    feats_header.push_back("gf_x_" + std::to_string(i));
    feats_header.push_back("gf_y_" + std::to_string(i));
    feats_header.push_back("gf_z_" + std::to_string(i));
    feats_header.push_back("un_" + std::to_string(i));
    feats_header.push_back("vn_" + std::to_string(i));
  }

  utils::SynteticDataParser data(dataset_path + "trajectory_imu.csv", dataset_path + "trajectory_gt.csv",
                                 dataset_path + "trajectory_feats.csv", imu_header, groundtruth_header, feats_header);

  data.parseAndCheck();

  const std::vector<std::string> results_titles = {
      "t",     "q_x",   "q_y",   "q_z",   "q_w",   "p_x",   "p_y",   "p_z",   "v_x",   "v_y",   "v_z",   "b_w_x",
      "b_w_y", "b_w_z", "b_a_x", "b_a_y", "b_a_z", "s_q_x", "s_q_y", "s_q_z", "s_q_w", "s_p_x", "s_p_y", "s_p_z"};

  utils::dataWriter result_writer(results_path + "synthetic.csv", results_titles, ",");

  msceqf::MSCEqF sys(std::string(argv[1]) + "/config/config.yaml");

  const auto timestamps = data.getSensorsTimestamps();
  const auto feats = data.getFeaturesData();
  for (const auto& timestamp : timestamps)
  {
    auto reading = data.consumeSensorReadingAt(timestamp);
    std::visit([&sys](auto&& arg) { sys.processMeasurement(arg); }, reading);
    if (std::holds_alternative<msceqf::TriangulatedFeatures>(reading))
    {
      auto est = sys.stateEstimate();
      result_writer << timestamp << est << std::endl;
    }
  }

  return 0;
}