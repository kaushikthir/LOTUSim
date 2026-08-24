#pragma once

#include <complex>
#include <cstddef>
#include <vector>

#include "fmcw_radar/FmcwRadarConfig.hpp"
#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

/*
 * One stationary scattering contribution entering one
 * azimuth's FMCW range-processing chain.
 */
struct RangeContribution {
    double rangeM{0.0};
    double receivedPowerW{0.0};
    double sampleWeight{0.0};
};

/*
 * Generates the range profile produced by a linear FMCW
 * beat tone after Hann windowing and range FFT processing.
 *
 * The Hann PSF coefficients are evaluated analytically.
 * They are not hand-selected blur weights.
 */
class FmcwRangeProcessor {
  public:
    explicit FmcwRangeProcessor(
        const FmcwRadarConfig& config);

    void process(
        AzimuthProfile& profile,
        const std::vector<RangeContribution>& contributions) const;

    double rangeBinSpacingM() const;

  private:
    std::complex<double> dirichletResponse(
        double binOffset) const;

    std::complex<double> periodicHannResponse(
        double binOffset) const;

    const FmcwRadarConfig& config_;

    std::size_t sampleCount_{0};
    std::size_t fftSize_{0};

    double rangeBinSpacingM_{0.0};
    double responseFloorLinearPower_{0.0};

    std::size_t maximumResponseHalfWidthBins_{0};
};

} // namespace fmcw_radar
