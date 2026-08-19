#include <fmcw_radar/scan/BeamGeometry.hpp>

#include <fmcw_radar/math/RadarMath.hpp>

namespace fmcw_radar {

BeamGeometry::BeamGeometry(const FmcwRadarConfig& config) : config_(config) {}

Ray BeamGeometry::makeRay(const RadarPose& pose, double scanAzimuthRad, const BeamSample& sample) const {
    Ray ray;

    ray.origin = pose.position;

    ray.direction = math::directionFromAngles(scanAzimuthRad + sample.azimuthOffsetRad,

                                              pose.pitchRad + sample.elevationOffsetRad);

    ray.maxRangeM = config_.maxRangeM;

    return ray;
}

} // namespace fmcw_radar
