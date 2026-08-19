#include <fmcw_radar/antenna/SincAntennaPattern.hpp>

#include <cmath>

namespace fmcw_radar {

namespace {

constexpr double HALF_POWER_ARGUMENT = 1.3915573782515103;

double sinc(double x) {
    if (std::abs(x) < 1e-12) {
        return 1.0;
    }

    return std::sin(x) / x;
}

} // namespace

SincAntennaPattern::SincAntennaPattern(double horizontalHpbwRad, double verticalHpbwRad)
    : horizontalHpbwRad_(horizontalHpbwRad),

      verticalHpbwRad_(verticalHpbwRad) {}

double SincAntennaPattern::powerGain(double azimuthOffsetRad, double elevationOffsetRad) const {
    const double horizontalScale = HALF_POWER_ARGUMENT / std::sin(horizontalHpbwRad_ / 2.0);

    const double verticalScale = HALF_POWER_ARGUMENT / std::sin(verticalHpbwRad_ / 2.0);

    const double horizontalAmplitude = sinc(horizontalScale * std::sin(azimuthOffsetRad));

    const double verticalAmplitude = sinc(verticalScale * std::sin(elevationOffsetRad));

    return horizontalAmplitude * horizontalAmplitude * verticalAmplitude * verticalAmplitude;
}

} // namespace fmcw_radar
