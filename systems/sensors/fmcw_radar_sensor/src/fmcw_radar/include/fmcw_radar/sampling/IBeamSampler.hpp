#pragma once

#include <vector>

#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

class IBeamSampler {
  public:
    virtual ~IBeamSampler() = default;

    virtual const std::vector<BeamSample>& samples() const = 0;
};

} // namespace fmcw_radar
