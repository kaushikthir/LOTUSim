#include <fmcw_radar/scattering/ConstantRcsScatter.hpp>

#include <algorithm>

namespace fmcw_radar {

ConstantRcsScatter::ConstantRcsScatter(double rcsM2) : rcsM2_(std::max(0.0, rcsM2)) {}

double ConstantRcsScatter::radarCrossSectionM2(const SurfaceHit& hit, const Vec3& incidentDirection, const Vec3& returnDirection, double frequencyHz) const {
    (void)hit;
    (void)incidentDirection;
    (void)returnDirection;
    (void)frequencyHz;

    return rcsM2_;
}

} // namespace fmcw_radar
