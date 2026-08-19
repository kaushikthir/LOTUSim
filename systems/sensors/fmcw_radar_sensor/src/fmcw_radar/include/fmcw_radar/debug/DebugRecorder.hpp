#pragma once

#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar::debug {

void recordRay(ScanDebug* debug,

               std::size_t azimuthIndex,

               double scanAzimuthRad,

               const BeamSample& beamSample,

               double antennaGain,

               const Ray& ray,

               bool didHit,

               const SurfaceHit& hit,

               double receivedPowerW);

}
