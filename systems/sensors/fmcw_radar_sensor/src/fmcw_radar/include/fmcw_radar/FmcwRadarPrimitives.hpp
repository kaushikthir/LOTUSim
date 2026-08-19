#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fmcw_radar {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 operator-() const {
        return {-x, -y, -z};
    }
};

struct Ray {
    Vec3 origin;
    Vec3 direction;
    double maxRangeM{0.0};
};

struct SurfaceHit {
    double distanceM{0.0};

    Vec3 position;
    Vec3 normal;

    std::int32_t objectId{-1};
    std::int32_t materialId{-1};
};

/*
 * Result for one ray in a batched scene query.
 */
struct RayTraceResult {
    bool hit{false};
    SurfaceHit surfaceHit;
};

struct RadarPose {
    Vec3 position;

    double yawRad{0.0};
    double pitchRad{0.0};
    double rollRad{0.0};
};

/*
 * One numerical sample used to represent
 * the continuous antenna response.
 *
 * This is NOT a physical electromagnetic wave.
 */
struct BeamSample {
    double azimuthOffsetRad{0.0};
    double elevationOffsetRad{0.0};

    /*
     * Numerical integration weight.
     */
    double weight{1.0};
};

struct AzimuthProfile {
    double timestampSeconds{0.0};
    double azimuthRad{0.0};
    std::vector<double> bins;
};

struct RadarFrame {
    double startTimestampSeconds{0.0};
    std::vector<AzimuthProfile> profiles;
};

/*
 * Optional information exposed for
 * visualisation/debugging.
 */
struct RaySample {
    std::size_t azimuthIndex{0};

    double scanAzimuthRad{0.0};
    double azimuthOffsetRad{0.0};
    double elevationOffsetRad{0.0};

    double antennaGain{0.0};
    double sampleWeight{0.0};

    Ray ray;

    bool hit{false};
    SurfaceHit surfaceHit;

    double receivedPowerW{0.0};
};

struct ScanDebug {
    std::vector<std::vector<RaySample>> raysByAzimuth;
};

} // namespace fmcw_radar
