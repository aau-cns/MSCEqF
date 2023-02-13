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
#include "utils/csv_parser.hpp"

// int main(int argc, char** argv)
int main()
{
  // if (argc != 2)
  // {
  //   std::cout << "Usage: ./euroc <dataset_name>" << std::endl;
  //   return 1;
  // }

  // const std::string dataset_name = argv[1];
  const std::string dataset_name = "V1_01_easy";
  const std::string dataset_path = "/media/alfornasier/PortableSSD/alfornasier/Datasets/Euroc/" + dataset_name;
  const std::string imu_path = dataset_path + "/mav0/imu0/data.csv";
  const std::string cam_path = dataset_path + "/mav0/cam0/data.csv";
  const std::string cam_image_path = dataset_path + "/mav0/cam0/data/";
  const std::string groundtruth_path = dataset_path + "/mav0/state_groundtruth_estimate0/data.csv";

  const std::vector<std::string> imu_header = {"#timestamp [ns]",     "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                                               "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]",   "a_RS_S_y [m s^-2]",
                                               "a_RS_S_z [m s^-2]"};
  const std::vector<std::string> groundtruth_header = {"#timestamp",
                                                       "q_RS_w []",
                                                       "q_RS_x []",
                                                       "q_RS_y []",
                                                       "q_RS_z []",
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

  utils::csvParser dataset_parser(imu_path, groundtruth_path, cam_path, cam_image_path, imu_header, groundtruth_header,
                                  cam_header);

  dataset_parser.parseAndCheck();

  msceqf::MSCEqF sys("/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/examples/euroc/config.yaml");

  const auto& gt = dataset_parser.getGroundtruthData();

  const auto timestamps = dataset_parser.getSensorsTimestamps();
  for (const auto& timestamp : timestamps)
  {
    const auto it =
        std::find_if(gt.begin(), gt.end(), [&timestamp](const auto& d) { return d.timestamp_ >= timestamp; });
    if (it != gt.end())
    {
      std::cout << "GT quaternion: " << it->q_.coeffs().transpose() << std::endl;
    }

    auto data = dataset_parser.consumeSensorReadingAt(timestamp);
    std::visit([&sys](auto&& arg) { sys.processMeasurement(arg); }, data);
  }

  return 0;
}