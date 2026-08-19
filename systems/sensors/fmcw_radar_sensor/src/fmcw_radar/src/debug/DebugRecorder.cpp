#include <fmcw_radar/debug/DebugRecorder.hpp>

namespace fmcw_radar::debug {

void recordRay(ScanDebug* debug,

               std::size_t azimuthIndex,

               double scanAzimuthRad,

               const BeamSample& beamSample,

               double antennaGain,

               const Ray& ray,

               bool didHit,

               const SurfaceHit& hit,

               double receivedPowerW) {
    if (debug == nullptr) {
        return;
    }

    RaySample event;

    event.azimuthIndex = azimuthIndex;

    event.scanAzimuthRad = scanAzimuthRad;

    event.azimuthOffsetRad = beamSample.azimuthOffsetRad;

    event.elevationOffsetRad = beamSample.elevationOffsetRad;

    event.antennaGain = antennaGain;

    event.sampleWeight = beamSample.weight;

    event.ray = ray;

    event.hit = didHit;

    if (didHit) {
        event.surfaceHit = hit;
    }

    event.receivedPowerW = receivedPowerW;

    debug->raysByAzimuth[azimuthIndex].push_back(event);
}

} // namespace fmcw_radar::debug
