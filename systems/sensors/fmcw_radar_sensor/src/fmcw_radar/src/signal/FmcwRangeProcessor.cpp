#include <algorithm>
#include <cmath>
#include <complex>
#include <fmcw_radar/signal/FmcwRangeProcessor.hpp>
#include <stdexcept>
#include <vector>

namespace fmcw_radar {

namespace {

constexpr double PI = 3.14159265358979323846;

constexpr double SPEED_OF_LIGHT_MPS = 299792458.0;

}  // namespace

FmcwRangeProcessor::FmcwRangeProcessor(const FmcwRadarConfig& config)
    : config_(config)
{
    const WaveformConfig& waveform = config_.waveform;

    if (waveform.model != "linear_fmcw") {
        throw std::invalid_argument(
            "Unsupported FMCW waveform model: " + waveform.model);
    }

    if (waveform.window != "hann_periodic") {
        throw std::invalid_argument("FmcwRangeProcessor currently supports "
                                    "window = hann_periodic");
    }

    if (!(waveform.bandwidthHz > 0.0) || !(waveform.chirpDurationS > 0.0)) {
        throw std::invalid_argument(
            "FMCW bandwidth and chirp duration must be positive");
    }

    if (waveform.adcSampleCount == 0 || waveform.fftSize == 0) {
        throw std::invalid_argument("FMCW ADC and FFT sizes must be non-zero");
    }

    /*
     * The analytical response currently assumes no zero padding.
     * Supporting a larger FFT is possible later, but requires a
     * different frequency-axis expression.
     */
    if (waveform.adcSampleCount != waveform.fftSize) {
        throw std::invalid_argument("Current FMCW range processor requires "
                                    "adc_sample_count == fft_size");
    }

    sampleCount_ = waveform.adcSampleCount;

    fftSize_ = waveform.fftSize;

    const double sampleRateHz =
        static_cast<double>(sampleCount_) / waveform.chirpDurationS;

    const double chirpSlopeHzPerSecond =
        waveform.bandwidthHz / waveform.chirpDurationS;

    /*
     * Beat-frequency FFT range spacing:
     *
     * delta_R = c fs / (2 S Nfft)
     */
    rangeBinSpacingM_ =
        SPEED_OF_LIGHT_MPS * sampleRateHz /
        (2.0 * chirpSlopeHzPerSecond * static_cast<double>(fftSize_));

    const double relativeSpacingError =
        std::abs(rangeBinSpacingM_ - config_.rangeBinSizeM) /
        config_.rangeBinSizeM;

    if (relativeSpacingError > 0.01) {
        throw std::invalid_argument(
            "Configured range_bin_size_m is inconsistent "
            "with the FMCW waveform by more than 1 percent");
    }

    const double maximumUnambiguousRangeM =
        rangeBinSpacingM_ * static_cast<double>(fftSize_);

    if (config_.maxRangeM > maximumUnambiguousRangeM) {
        throw std::invalid_argument("Configured maximum range exceeds the "
                                    "FMCW unambiguous range");
    }

    responseFloorLinearPower_ = std::pow(10.0, waveform.responseFloorDb / 10.0);

    maximumResponseHalfWidthBins_ =
        std::min(waveform.maximumResponseHalfWidthBins, fftSize_ / 2);
}

double FmcwRangeProcessor::rangeBinSpacingM() const
{
    return rangeBinSpacingM_;
}

std::complex<double> FmcwRangeProcessor::dirichletResponse(
    double binOffset) const
{
    /*
     * D_N(x) =
     *   exp(j pi x (N-1)/N)
     *   sin(pi x) / sin(pi x/N)
     *
     * This is the exact DFT response of a length-N
     * complex sinusoid at a fractional bin offset x.
     */
    const double nearestInteger = std::round(binOffset);

    if (std::abs(binOffset - nearestInteger) < 1.0e-11) {
        const long long integerOffset =
            static_cast<long long>(std::llround(nearestInteger));

        const long long count = static_cast<long long>(sampleCount_);

        const long long wrapped = ((integerOffset % count) + count) % count;

        if (wrapped == 0) {
            return {static_cast<double>(sampleCount_), 0.0};
        }

        return {0.0, 0.0};
    }

    const double denominator =
        std::sin(PI * binOffset / static_cast<double>(sampleCount_));

    if (std::abs(denominator) < 1.0e-15) {
        return {0.0, 0.0};
    }

    const double magnitude = std::sin(PI * binOffset) / denominator;

    const double phase = PI * binOffset *
                         static_cast<double>(sampleCount_ - 1) /
                         static_cast<double>(sampleCount_);

    return std::polar(magnitude, phase);
}

std::complex<double> FmcwRangeProcessor::periodicHannResponse(
    double binOffset) const
{
    /*
     * Periodic Hann:
     *
     * w[n] = 0.5 - 0.5 cos(2 pi n/N)
     *
     * Its exact DFT response is a combination of three
     * shifted Dirichlet responses.
     */
    const std::complex<double> response =
        0.5 * dirichletResponse(binOffset) -
        0.25 * dirichletResponse(binOffset + 1.0) -
        0.25 * dirichletResponse(binOffset - 1.0);

    /*
     * Energy normalization.
     *
     * For a periodic Hann window:
     * sum(w[n]^2) = 3N/8.
     *
     * This normalization preserves the contribution's
     * total spectral power across the complete FFT.
     */
    const double normalization =
        static_cast<double>(sampleCount_) * std::sqrt(3.0 / 8.0);

    return response / normalization;
}

void FmcwRangeProcessor::process(
    AzimuthProfile& profile,
    const std::vector<RangeContribution>& contributions) const
{
    std::fill(profile.bins.begin(), profile.bins.end(), 0.0);

    if (profile.bins.empty() || contributions.empty()) {
        return;
    }

    std::vector<std::complex<double>> spectrum(profile.bins.size(), {0.0, 0.0});

    const double wavelengthM = SPEED_OF_LIGHT_MPS / config_.carrierFrequencyHz;

    const int halfWidth = static_cast<int>(maximumResponseHalfWidthBins_);

    for (const RangeContribution& contribution : contributions) {
        if (!(contribution.rangeM >= 0.0) ||
            contribution.rangeM > config_.maxRangeM ||
            !(contribution.receivedPowerW > 0.0) ||
            !(contribution.sampleWeight > 0.0) ||
            !std::isfinite(contribution.rangeM) ||
            !std::isfinite(contribution.receivedPowerW) ||
            !std::isfinite(contribution.sampleWeight)) {
            continue;
        }

        const double effectivePowerW =
            contribution.receivedPowerW * contribution.sampleWeight;

        const double amplitude = std::sqrt(effectivePowerW);

        /*
         * Fractional FFT-bin location of this target.
         * A non-integer position naturally produces leakage
         * according to the selected FFT window.
         */
        const double targetBin = contribution.rangeM / rangeBinSpacingM_;

        const long long centreBin = std::llround(targetBin);

        /*
         * Round-trip carrier phase:
         *
         * phi = -4 pi R / lambda
         */
        const double roundTripPhase = std::remainder(
            -4.0 * PI * contribution.rangeM / wavelengthM,
            2.0 * PI);

        const std::complex<double> targetAmplitude =
            std::polar(amplitude, roundTripPhase);

        for (int offset = -halfWidth; offset <= halfWidth; ++offset) {
            const long long outputBin =
                centreBin + static_cast<long long>(offset);

            if (outputBin < 0 ||
                outputBin >= static_cast<long long>(profile.bins.size())) {
                continue;
            }

            const double binOffset = targetBin - static_cast<double>(outputBin);

            const std::complex<double> response =
                periodicHannResponse(binOffset);

            const double responsePower = std::norm(response);

            if (responsePower < responseFloorLinearPower_) {
                continue;
            }

            spectrum[static_cast<std::size_t>(outputBin)] +=
                targetAmplitude * response;
        }
    }

    for (std::size_t bin = 0; bin < profile.bins.size(); ++bin) {
        profile.bins[bin] = std::norm(spectrum[bin]);
    }
}

}  // namespace fmcw_radar
