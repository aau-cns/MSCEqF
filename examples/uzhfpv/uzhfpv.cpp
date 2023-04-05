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
    std::cout << "Usage: ./msceqf_uzhfpv <dataset_name> <dataset_folder> <uzhfpv_folder> Each without / at the end."
              << std::endl;
    return 1;
  }

  const std::string dataset_name = argv[1];
  const std::string dataset_path = std::string(argv[2]) + "/" + dataset_name;
  const std::string results_path = std::string(argv[3]) + "/results/" + dataset_name + ".csv";
  const std::string imu_path = dataset_path + "/imu.txt";
  const std::string cam_path = dataset_path + "/left_images.txt";
  const std::string cam_image_path = dataset_path + "/";
  const std::string groundtruth_path = dataset_path + "/groundtruth.txt";

  const std::vector<std::string> imu_header = {"timestamp", "ang_vel_x", "ang_vel_y", "ang_vel_z",
                                               "lin_acc_x", "lin_acc_y", "lin_acc_z"};

  const std::vector<std::string> groundtruth_header = {"timestamp", "qx", "qy", "qz", "qw", "tx", "ty", "tz"};

  const std::vector<std::string> cam_header = {"timestamp", "image_name"};

  const std::vector<std::string> results_titles = {
      "t",     "q_x",   "q_y",   "q_z",   "q_w",   "p_x",   "p_y",   "p_z",   "v_x",   "v_y",   "v_z",   "b_w_x",
      "b_w_y", "b_w_z", "b_a_x", "b_a_y", "b_a_z", "s_q_x", "s_q_y", "s_q_z", "s_q_w", "s_p_x", "s_p_y", "s_p_z"};

  utils::dataParser dataset_parser(imu_path, groundtruth_path, cam_path, cam_image_path, imu_header, groundtruth_header,
                                   cam_header, ' ', -0.016684572091862235);
  dataset_parser.parseAndCheck();

  utils::dataWriter result_writer(results_path, results_titles, ",");

  msceqf::MSCEqF sys(std::string(argv[3]) + "/config/config.yaml");

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