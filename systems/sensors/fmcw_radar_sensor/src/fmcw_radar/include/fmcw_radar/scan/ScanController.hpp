#pragma once

#include <cstddef>

#include "fmcw_radar/FmcwRadarConfig.hpp"
#include "fmcw_radar/FmcwRadarPrimitives.hpp"
#include "fmcw_radar/noise/INoiseModel.hpp"

namespace fmcw_radar {

struct ScanStep {
    std::size_t index{0};

    double azimuthRad{0.0};

    double timestampSeconds{0.0};
};

class ScanController {
  public:
    explicit ScanController(const FmcwRadarConfig& config);

    ScanStep step(std::size_t azimuthIndex, const RadarPose& pose, double scanStartTime) const;

    NoiseContext noiseContext(const ScanStep& step) const;

  private:
    const FmcwRadarConfig& config_;
};

} // namespace fmcw_radar
