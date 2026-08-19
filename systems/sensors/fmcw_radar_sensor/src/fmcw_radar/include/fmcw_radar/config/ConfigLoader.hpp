#pragma once

#include <string>

#include "fmcw_radar/FmcwRadarConfig.hpp"

namespace fmcw_radar {

FmcwRadarConfig loadRadarConfig(const std::string& filename);

}
