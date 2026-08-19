#pragma once

#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

class IScatterModel {
  public:
    virtual ~IScatterModel() = default;

    virtual double radarCrossSectionM2(const SurfaceHit& hit, const Vec3& incidentDirection, const Vec3& returnDirection, double frequencyHz) const = 0;
};

} // namespace fmcw_radar
