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

#ifndef TEST_PARAMS_HPP
#define TEST_PARAMS_HPP

#include <iostream>

#include "msceqf/options/msceqf_option_parser.hpp"

namespace msceqf
{

TEST(OptionParserTest, parse)
{
  std::string filepath_base = "/home/alfornasier/PhD/MSCEqF_dev/MSCEqF/config/";
  int num_files_with_missing = 3;

  {
    OptionParser parser(filepath_base + "parameters.yaml");
    [[maybe_unused]] MSCEqFOptions opts = parser.parseOptions();
  }

  for (int i = 1; i <= num_files_with_missing; ++i)
  {
    OptionParser parser(filepath_base + "parameters_missing_" + std::to_string(i) + ".yaml");
    EXPECT_THROW({ [[maybe_unused]] MSCEqFOptions opts = parser.parseOptions(); }, std::runtime_error);
  }
}

}  // namespace msceqf

#endif  // TEST_PARAMS_HPP