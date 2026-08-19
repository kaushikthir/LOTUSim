#include <fmcw_radar/propagation/FreeSpacePropagation.hpp>

#include <fmcw_radar/math/RadarMath.hpp>

namespace fmcw_radar {

double FreeSpacePropagation::oneWayFactor(double distanceM, double frequencyHz) const {
    (void)frequencyHz;

    if (distanceM <= 0.0) {
        return 0.0;
    }

    return 1.0 / (4.0 * math::PI * distanceM * distanceM);
}

} // namespace fmcw_radar
