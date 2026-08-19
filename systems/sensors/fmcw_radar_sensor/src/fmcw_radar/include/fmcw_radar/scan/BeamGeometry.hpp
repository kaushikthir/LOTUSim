#pragma once

#include "fmcw_radar/FmcwRadarConfig.hpp"
#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

class BeamGeometry {
  public:
    explicit BeamGeometry(const FmcwRadarConfig& config);

    Ray makeRay(const RadarPose& pose, double scanAzimuthRad, const BeamSample& sample) const;

  private:
    const FmcwRadarConfig& config_;
};

} // namespace fmcw_radar
