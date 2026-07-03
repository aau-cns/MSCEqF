// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#ifndef TEST_PROPAGATION_HPP
#define TEST_PROPAGATION_HPP

#include "msceqf/msceqf.hpp"
#include "utils/data_parser.hpp"

/**
 * @brief This test checks the IMU data selection in the propagation phase.
 *
 * 1) Camera timestamp in between Imu timestamps
 * 2) Camera timestamp equal to a Imu timestamp in the middle
 * 3) Camera timestamp equal to the last Imu timestamp
 * 4) Camera timestamp grater than the last Imu timestamp
 * 5) Camera timestamp grater than the first Imu timestamp but smaller than the second
 * 6) Camera timestamp equal to the first Imu timestamp
 *
 * @note In order to inspect the IMU buffer, uncomment debug output in source/msceqf/filter/propagator/propagator.cpp
 */

namespace msceqf
{

class MSCEqFPropagationImuMeasurementSelectionTest : public ::testing::TestWithParam<fp>
{
 public:
  void triggerPropagationAt(const fp& t)
  {
    MSCEqF sys(parameters_path);

    for (int i = 0; i < 100; ++i)
    {
      Imu imu;
      imu.timestamp_ = i * 0.1;
      sys.processMeasurement(imu);
    }

    Camera cam;
    cam.timestamp_ = t;
    sys.processMeasurement(cam);
  }
};

TEST_P(MSCEqFPropagationImuMeasurementSelectionTest, ImuMeasurementSelection)
{
  fp timestamp = GetParam();
  triggerPropagationAt(timestamp);
}

INSTANTIATE_TEST_CASE_P(ImuMeasurementSelectionTest,
                        MSCEqFPropagationImuMeasurementSelectionTest,
                        ::testing::Values(7.05, 5.0, 9.9, 11, 0.05, 0.0));

}  // namespace msceqf

#endif  // TEST_PROPAGATION_HPP