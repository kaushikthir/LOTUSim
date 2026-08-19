#pragma once

#include <memory>
#include <string>

#include "fmcw_radar/IFmcwRadarScene.hpp"
#include "fmcw_radar/FmcwRadarConfig.hpp"
#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

class IAntennaPattern;
class IPropagationModel;
class IScatterModel;
class IBeamSampler;
class INoiseModel;

class FmcwRadar {
  public:
    explicit FmcwRadar(const std::string& configFile);

    explicit FmcwRadar(const FmcwRadarConfig& config);

    ~FmcwRadar();

    FmcwRadar(FmcwRadar&&) noexcept;

    FmcwRadar& operator=(FmcwRadar&&) noexcept;

    FmcwRadar(const FmcwRadar&) = delete;

    FmcwRadar& operator=(const FmcwRadar&) = delete;

    /*
     * Main radar entry point.
     */
    RadarFrame scan(const IFmcwRadarScene& scene, const RadarPose& pose, double timestampSeconds, ScanDebug* debug = nullptr) const;

    const FmcwRadarConfig& config() const;

  private:
    FmcwRadarConfig config_;

    std::unique_ptr<IAntennaPattern> antenna_;

    std::unique_ptr<IPropagationModel> propagation_;

    std::unique_ptr<IScatterModel> scattering_;

    std::unique_ptr<IBeamSampler> sampler_;

    std::unique_ptr<INoiseModel> noise_;
};

} // namespace fmcw_radar
