#include "fmcw_radar/fmcw_radar.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <fmcw_radar/config/ConfigLoader.hpp>
#include <fmcw_radar/debug/DebugRecorder.hpp>
#include <fmcw_radar/factory/ModelFactory.hpp>
#include <fmcw_radar/output/RangeBinner.hpp>
#include <fmcw_radar/response/RadarReturnModel.hpp>
#include <fmcw_radar/scan/BeamGeometry.hpp>
#include <fmcw_radar/scan/ScanController.hpp>

namespace fmcw_radar {

// ============================================================
// ASSEMBLE RADAR
// ============================================================

FmcwRadar::FmcwRadar(const std::string& configFile)
    : FmcwRadar(loadRadarConfig(configFile)) {
}

FmcwRadar::FmcwRadar(const FmcwRadarConfig& config)
    : config_(config) {
    antenna_ = makeAntennaPattern(config_);
    propagation_ = makePropagationModel(config_);
    scattering_ = makeScatterModel(config_);
    sampler_ = makeBeamSampler(config_);
    noise_ = makeNoiseModel(config_);
}

FmcwRadar::~FmcwRadar() = default;

FmcwRadar::FmcwRadar(FmcwRadar&&) noexcept = default;

FmcwRadar& FmcwRadar::operator=(FmcwRadar&&) noexcept = default;


// ============================================================
// RADAR BRAIN
// ============================================================

RadarFrame FmcwRadar::scan(
    const IFmcwRadarScene& scene,
    const RadarPose& pose,
    double timestampSeconds,
    ScanDebug* debug) const {

    ScanController scanController(config_);
    BeamGeometry beamGeometry(config_);
    RadarReturnModel returnModel(
        config_,
        *propagation_,
        *scattering_);
    RangeBinner rangeBinner(config_);

    RadarFrame frame;
    frame.startTimestampSeconds = timestampSeconds;
    frame.profiles.reserve(config_.azimuthCount);

    if (debug != nullptr) {
        debug->raysByAzimuth.clear();
        debug->raysByAzimuth.resize(config_.azimuthCount);
    }

    const std::vector<BeamSample>& beamSamples =
        sampler_->samples();

    const std::size_t totalRayCount =
        config_.azimuthCount * beamSamples.size();

    /*
     * First generate the whole radar scan.
     *
     * This lets an accelerated scene backend process all rays
     * as one batch rather than receiving 219,600 separate calls.
     */
    std::vector<ScanStep> steps;
    std::vector<AzimuthProfile> profiles;
    std::vector<Ray> rays;
    std::vector<double> antennaGains;

    steps.reserve(config_.azimuthCount);
    profiles.reserve(config_.azimuthCount);
    rays.reserve(totalRayCount);
    antennaGains.reserve(totalRayCount);

    for (std::size_t azimuthIndex = 0;
         azimuthIndex < config_.azimuthCount;
         ++azimuthIndex) {

        const ScanStep step =
            scanController.step(
                azimuthIndex,
                pose,
                timestampSeconds);

        steps.push_back(step);

        profiles.push_back(
            rangeBinner.makeProfile(
                step.azimuthRad,
                step.timestampSeconds));

        for (const BeamSample& sample : beamSamples) {
            antennaGains.push_back(
                antenna_->powerGain(
                    sample.azimuthOffsetRad,
                    sample.elevationOffsetRad));

            rays.push_back(
                beamGeometry.makeRay(
                    pose,
                    step.azimuthRad,
                    sample));
        }
    }

    const std::vector<RayTraceResult> traceResults =
        scene.traceNearestBatch(rays);

    if (traceResults.size() != rays.size()) {
        throw std::runtime_error(
            "Scene batch result count does not match radar ray count");
    }

    /*
     * Convert the geometry hits into received radar power
     * and then place that power into range bins.
     */
    std::size_t rayIndex = 0;

    for (std::size_t azimuthIndex = 0;
         azimuthIndex < config_.azimuthCount;
         ++azimuthIndex) {

        AzimuthProfile& profile = profiles[azimuthIndex];
        const ScanStep& step = steps[azimuthIndex];

        for (const BeamSample& sample : beamSamples) {
            const Ray& ray = rays[rayIndex];
            const double antennaGain =
                antennaGains[rayIndex];

            const RayTraceResult& traceResult =
                traceResults[rayIndex];

            double receivedPower = 0.0;

            if (traceResult.hit) {
                receivedPower =
                    returnModel.calculate(
                        ray,
                        traceResult.surfaceHit,
                        antennaGain);

                rangeBinner.add(
                    profile,
                    traceResult.surfaceHit.distanceM,
                    receivedPower,
                    sample.weight);
            }

            debug::recordRay(
                debug,
                azimuthIndex,
                step.azimuthRad,
                sample,
                antennaGain,
                ray,
                traceResult.hit,
                traceResult.surfaceHit,
                receivedPower);

            ++rayIndex;
        }

        noise_->apply(
            profile,
            scanController.noiseContext(step));

        frame.profiles.push_back(std::move(profile));
    }

    return frame;
}


// ============================================================
// CONFIG ACCESS
// ============================================================

const FmcwRadarConfig& FmcwRadar::config() const {
    return config_;
}

} // namespace fmcw_radar
