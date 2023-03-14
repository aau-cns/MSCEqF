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

// int main(int argc, char** argv)
int main()
{
  // const std::string dataset_name = argv[1];
  const std::string dataset_path = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/examples/syntetic/trajectory.csv";
  const std::string results_path = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/examples/syntetic/results.csv";

  const std::vector<std::string> imu_header = {"t", "w_x", "w_y", "w_z", "a_x", "a_y", "a_z"};

  const std::vector<std::string> groundtruth_header = {"t",   "q_x", "q_y", "q_z", "q_w", "p_x",
                                                       "p_y", "p_z", "v_x", "v_y", "v_z"};

  std::vector<std::string> feats_header;
  for (int i = 1; i <= 500; ++i)
  {
    feats_header.push_back("Gf_x_" + std::to_string(i));
    feats_header.push_back("Gf_y_" + std::to_string(i));
    feats_header.push_back("Gf_z_" + std::to_string(i));
    feats_header.push_back("un_" + std::to_string(i));
    feats_header.push_back("vn_" + std::to_string(i));
  }

  utils::dataParser dataset_parser(dataset_path, dataset_path, "", "", dataset_path, imu_header, groundtruth_header, {},
                                   feats_header);
  dataset_parser.parseAndCheck();

  const std::vector<std::string> results_titles = {
      "t",     "q_x",   "q_y",   "q_z",   "q_w",   "p_x",   "p_y",   "p_z",   "v_x",   "v_y",   "v_z",   "b_w_x",
      "b_w_y", "b_w_z", "b_a_x", "b_a_y", "b_a_z", "s_q_x", "s_q_y", "s_q_z", "s_q_w", "s_p_x", "s_p_y", "s_p_z"};

  utils::dataWriter result_writer(results_path, results_titles, ",");

  msceqf::MSCEqF sys("/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/examples/euroc/config.yaml");

  const auto timestamps = dataset_parser.getSensorsTimestamps();
  for (const auto& timestamp : timestamps)
  {
    auto data = dataset_parser.consumeSensorReadingAt(timestamp);
    std::visit([&sys](auto&& arg) { sys.processMeasurement(arg); }, data);

    if (std::holds_alternative<msceqf::Camera>(data))
    {
      auto est = sys.stateEstimate();
      result_writer << timestamp << est << std::endl;
    }
  }

  return 0;
}