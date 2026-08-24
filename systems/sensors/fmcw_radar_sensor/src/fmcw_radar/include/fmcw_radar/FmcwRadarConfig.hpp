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


struct WaveformConfig {
    /*
     * Linear FMCW waveform and range-processing configuration.
     */
    std::string model{"linear_fmcw"};

    double bandwidthHz{967072445.1612903};
    double chirpDurationS{1.0e-4};

    std::size_t adcSampleCount{8192};
    std::size_t fftSize{8192};

    std::string window{"hann_periodic"};

    /*
     * Numerical truncation controls.
     *
     * These do not define the PSF shape. The PSF coefficients
     * are calculated from the Hann-window Fourier response.
     */
    double responseFloorDb{-60.0};
    std::size_t maximumResponseHalfWidthBins{32};
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
    // FMCW WAVEFORM AND RANGE PROCESSING
    // ========================================================

    WaveformConfig waveform;

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
