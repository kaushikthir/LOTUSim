#pragma once

#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar::math {

constexpr double PI = 3.14159265358979323846;

constexpr double SPEED_OF_LIGHT_MPS = 299792458.0;

double degToRad(double degrees);

double wavelengthM(double frequencyHz);

/*
 * Converts radar azimuth/elevation angles
 * into a unit direction vector.
 *
 * Convention:
 *
 * azimuth = 0      -> -Z
 * azimuth = +90°   -> +X
 * elevation = +    -> +Y
 */
Vec3 directionFromAngles(double azimuthRad, double elevationRad);

} // namespace fmcw_radar::math
