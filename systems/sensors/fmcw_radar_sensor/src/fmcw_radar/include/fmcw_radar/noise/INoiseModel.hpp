#pragma once

#include <cstddef>

#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

struct NoiseContext {
    std::size_t azimuthIndex{0};

    double timestampSeconds{0.0};
    double azimuthRad{0.0};

    double carrierFrequencyHz{0.0};
    double rangeBinSizeM{0.0};
};

class INoiseModel {
  public:
    virtual ~INoiseModel() = default;

    virtual void apply(AzimuthProfile& profile, const NoiseContext& context) const = 0;
};

} // namespace fmcw_radar
