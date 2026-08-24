#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace fmcw_radar {

struct NoiseConfig {
    /*
     * Current:
     *     "none"
     *
     * Possible future models:
     *     "gaussian"
     *     "thermal"
     *     "receiver"
     *     "measured"
     */
    std::string model{"none"};

    /*
     * Allows stochastic models to be
     * repeatable when required.
     */
    std::uint64_t seed{0};

    /*
     * Parameters specific to whichever
     * noise model is selected.
     */
    std::unordered_map<std::string, double> parameters;
};

struct FmcwRadarConfig {
    std::string name;

    // ========================================================
    // SENSOR
    // ========================================================

    double carrierFrequencyHz{77e9};

    double maxRangeM{1000.0};

    double rangeBinSizeM{0.155};

    std::size_t azimuthCount{400};

    double rotationRateHz{2.0};

    // ========================================================
    // ANTENNA
    // ========================================================

    std::string antennaModel{"sinc"};

    double horizontalBeamwidthDeg{1.8};

    double verticalBeamwidthDeg{16.0};

    double boresightGainLinear{100.0};

    // ========================================================
    // TRANSMITTER
    // ========================================================

    double transmitPowerW{1.0};

    // ========================================================
    // TARGET SCATTERING
    // ========================================================

    std::string scatteringModel{"constant_rcs"};

    double constantRcsM2{10.0};

    // ========================================================
    // NUMERICAL ANTENNA SAMPLING
    // ========================================================

    std::size_t horizontalSamples{61};

    std::size_t verticalSamples{9};

    double horizontalSampleSpanDeg{6.0};

    double verticalSampleSpanDeg{8.0};

    // ========================================================
    // NOISE
    //
    // Current baseline:
    //     model = "none"
    // ========================================================

    NoiseConfig noise;
};

} // namespace fmcw_radar
