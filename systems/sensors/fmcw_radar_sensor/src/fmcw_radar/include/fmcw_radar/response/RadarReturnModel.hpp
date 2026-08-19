#pragma once

#include "fmcw_radar/FmcwRadarConfig.hpp"
#include "fmcw_radar/FmcwRadarPrimitives.hpp"

#include "fmcw_radar/propagation/IPropagationModel.hpp"
#include "fmcw_radar/scattering/IScatterModel.hpp"

namespace fmcw_radar {

class RadarReturnModel {
  public:
    RadarReturnModel(const FmcwRadarConfig& config, const IPropagationModel& propagation, const IScatterModel& scattering);

    double calculate(const Ray& ray, const SurfaceHit& hit, double antennaGain) const;

  private:
    const FmcwRadarConfig& config_;

    const IPropagationModel& propagation_;

    const IScatterModel& scattering_;

    double receiveApertureM2_;
};

} // namespace fmcw_radar
