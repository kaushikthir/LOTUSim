#include <fmcw_radar/factory/ModelFactory.hpp>

#include <stdexcept>

#include <fmcw_radar/antenna/SincAntennaPattern.hpp>
#include <fmcw_radar/noise/NoNoise.hpp>
#include <fmcw_radar/propagation/FreeSpacePropagation.hpp>
#include <fmcw_radar/sampling/GridBeamSampler.hpp>
#include <fmcw_radar/scattering/ConstantRcsScatter.hpp>

#include <fmcw_radar/math/RadarMath.hpp>

namespace fmcw_radar {

std::unique_ptr<IAntennaPattern> makeAntennaPattern(const FmcwRadarConfig& config) {
    if (config.antennaModel == "sinc") {
        return std::make_unique<SincAntennaPattern>(math::degToRad(config.horizontalBeamwidthDeg),

                                                    math::degToRad(config.verticalBeamwidthDeg));
    }

    throw std::runtime_error("Unsupported antenna model: " + config.antennaModel);
}

std::unique_ptr<IPropagationModel> makePropagationModel(const FmcwRadarConfig& config) {
    (void)config;

    return std::make_unique<FreeSpacePropagation>();
}

std::unique_ptr<IScatterModel> makeScatterModel(const FmcwRadarConfig& config) {
    if (config.scatteringModel == "constant_rcs") {
        return std::make_unique<ConstantRcsScatter>(config.constantRcsM2);
    }

    throw std::runtime_error("Unsupported scattering model: " + config.scatteringModel);
}

std::unique_ptr<IBeamSampler> makeBeamSampler(const FmcwRadarConfig& config) {
    return std::make_unique<GridBeamSampler>(math::degToRad(config.horizontalSampleSpanDeg),

                                             math::degToRad(config.verticalSampleSpanDeg),

                                             config.horizontalSamples,

                                             config.verticalSamples);
}

std::unique_ptr<INoiseModel> makeNoiseModel(const FmcwRadarConfig& config) {
    if (config.noise.model == "none") {
        return std::make_unique<NoNoise>();
    }

    throw std::runtime_error("Unsupported noise model: " + config.noise.model);
}

} // namespace fmcw_radar
