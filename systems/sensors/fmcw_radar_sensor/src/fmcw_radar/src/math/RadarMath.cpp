#include <fmcw_radar/math/RadarMath.hpp>

#include <cmath>
#include <stdexcept>

namespace fmcw_radar::math {

double degToRad(double degrees) {
    return degrees * PI / 180.0;
}

double wavelengthM(double frequencyHz) {
    if (frequencyHz <= 0.0) {
        throw std::runtime_error("Frequency must be positive");
    }

    return SPEED_OF_LIGHT_MPS / frequencyHz;
}

Vec3 directionFromAngles(double azimuthRad, double elevationRad) {
    const double cosElevation = std::cos(elevationRad);

    return {cosElevation * std::sin(azimuthRad),

            std::sin(elevationRad),

            -cosElevation * std::cos(azimuthRad)};
}

} // namespace fmcw_radar::math
