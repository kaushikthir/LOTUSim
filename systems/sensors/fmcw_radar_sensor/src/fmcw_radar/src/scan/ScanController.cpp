#include <fmcw_radar/scan/ScanController.hpp>

#include <fmcw_radar/math/RadarMath.hpp>

namespace fmcw_radar {

ScanController::ScanController(const FmcwRadarConfig& config) : config_(config) {}

ScanStep ScanController::step(std::size_t azimuthIndex, const RadarPose& pose, double scanStartTime) const {
    const double rotationPeriod = 1.0 / config_.rotationRateHz;

    ScanStep result;

    result.index = azimuthIndex;

    result.azimuthRad = pose.yawRad + 2.0 * math::PI * static_cast<double>(azimuthIndex) / static_cast<double>(config_.azimuthCount);

    result.timestampSeconds = scanStartTime + rotationPeriod * static_cast<double>(azimuthIndex) / static_cast<double>(config_.azimuthCount);

    return result;
}

NoiseContext ScanController::noiseContext(const ScanStep& step) const {
    NoiseContext context;

    context.azimuthIndex = step.index;

    context.timestampSeconds = step.timestampSeconds;

    context.azimuthRad = step.azimuthRad;

    context.carrierFrequencyHz = config_.carrierFrequencyHz;

    context.rangeBinSizeM = config_.rangeBinSizeM;

    return context;
}

} // namespace fmcw_radar
