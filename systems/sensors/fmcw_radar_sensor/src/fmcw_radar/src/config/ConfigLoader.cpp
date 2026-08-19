#include <fmcw_radar/config/ConfigLoader.hpp>

#include <cstdint>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace fmcw_radar {

FmcwRadarConfig loadRadarConfig(const std::string& filename) {
    std::ifstream input(filename);

    if (!input) {
        throw std::runtime_error("Could not open radar config: " + filename);
    }

    nlohmann::json j;

    input >> j;

    FmcwRadarConfig config;

    config.name = j.at("name").get<std::string>();

    // ========================================================
    // SENSOR
    // ========================================================

    const auto& sensor = j.at("sensor");

    config.carrierFrequencyHz = sensor.at("carrier_frequency_hz").get<double>();

    config.maxRangeM = sensor.at("max_range_m").get<double>();

    config.rangeBinSizeM = sensor.at("range_bin_size_m").get<double>();

    config.azimuthCount = sensor.at("azimuth_count").get<std::size_t>();

    config.rotationRateHz = sensor.at("rotation_rate_hz").get<double>();

    // ========================================================
    // ANTENNA
    // ========================================================

    const auto& antenna = j.at("antenna");

    config.antennaModel = antenna.at("model").get<std::string>();

    config.horizontalBeamwidthDeg = antenna.at("horizontal_beamwidth_deg").get<double>();

    config.verticalBeamwidthDeg = antenna.at("vertical_beamwidth_deg").get<double>();

    // ========================================================
    // SIMULATION ASSUMPTIONS
    // ========================================================

    const auto& assumptions = j.at("simulation_assumptions");

    config.transmitPowerW = assumptions.at("transmit_power_w").get<double>();

    config.boresightGainLinear = assumptions.at("boresight_gain_linear").get<double>();

    config.scatteringModel = assumptions.at("scattering_model").get<std::string>();

    config.constantRcsM2 = assumptions.at("constant_rcs_m2").get<double>();

    // ========================================================
    // NUMERICAL SAMPLING
    // ========================================================

    const auto& sampling = j.at("numerical_sampling");

    config.horizontalSamples = sampling.at("horizontal_samples").get<std::size_t>();

    config.verticalSamples = sampling.at("vertical_samples").get<std::size_t>();

    config.horizontalSampleSpanDeg = sampling.at("horizontal_half_span_deg").get<double>();

    config.verticalSampleSpanDeg = sampling.at("vertical_half_span_deg").get<double>();

    // ========================================================
    // NOISE
    // ========================================================

    if (j.contains("noise")) {
        const auto& noise = j.at("noise");

        config.noise.model = noise.value("model", std::string("none"));

        config.noise.seed = noise.value("seed", std::uint64_t{0});

        if (noise.contains("parameters")) {
            for (const auto& item : noise.at("parameters").items()) {
                config.noise.parameters[item.key()] = item.value().get<double>();
            }
        }
    }

    return config;
}

} // namespace fmcw_radar
