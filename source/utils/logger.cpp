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

#include "utils/logger.hpp"

namespace utils
{
const LoggerLevel& Logger::getlevel() { return level_; }

void Logger::setLevel(const LoggerLevel& level) { level_ = level; }

LoggerLevel Logger::level_ = LoggerLevel::FULL;

}  // namespace utils