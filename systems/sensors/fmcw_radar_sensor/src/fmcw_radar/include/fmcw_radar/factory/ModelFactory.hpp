#pragma once

#include <memory>

#include "fmcw_radar/FmcwRadarConfig.hpp"

#include "fmcw_radar/antenna/IAntennaPattern.hpp"
#include "fmcw_radar/noise/INoiseModel.hpp"
#include "fmcw_radar/propagation/IPropagationModel.hpp"
#include "fmcw_radar/sampling/IBeamSampler.hpp"
#include "fmcw_radar/scattering/IScatterModel.hpp"

namespace fmcw_radar {

std::unique_ptr<IAntennaPattern> makeAntennaPattern(const FmcwRadarConfig& config);

std::unique_ptr<IPropagationModel> makePropagationModel(const FmcwRadarConfig& config);

std::unique_ptr<IScatterModel> makeScatterModel(const FmcwRadarConfig& config);

std::unique_ptr<IBeamSampler> makeBeamSampler(const FmcwRadarConfig& config);

std::unique_ptr<INoiseModel> makeNoiseModel(const FmcwRadarConfig& config);

} // namespace fmcw_radar
