#include <fmcw_radar/output/RangeBinner.hpp>

#include <array>
#include <cmath>
#include <cstddef>

namespace fmcw_radar {

namespace {

/*
 * Temporary power-domain range PSF used for the first test.
 *
 * A return is distributed across:
 *
 *     centre - 2 bins :  5%
 *     centre - 1 bin  : 20%
 *     centre bin      : 50%
 *     centre + 1 bin  : 20%
 *     centre + 2 bins :  5%
 *
 * The coefficients sum to 1.0, so total received power
 * is preserved.
 *
 * This is currently a test kernel, not a validated MAS10 PSF.
 */
constexpr std::array<double, 5> RANGE_PSF{
    0.05,
    0.20,
    0.50,
    0.20,
    0.05
};

constexpr std::ptrdiff_t RANGE_PSF_CENTRE = 2;

} // namespace

RangeBinner::RangeBinner(const FmcwRadarConfig& config)
    : config_(config) {

    binCount_ = static_cast<std::size_t>(
        std::ceil(
            config_.maxRangeM /
            config_.rangeBinSizeM));
}

AzimuthProfile RangeBinner::makeProfile(
    double azimuthRad,
    double timestampSeconds) const {

    AzimuthProfile profile;

    profile.azimuthRad = azimuthRad;
    profile.timestampSeconds = timestampSeconds;
    profile.bins.assign(binCount_, 0.0);

    return profile;
}

void RangeBinner::add(
    AzimuthProfile& profile,
    double rangeM,
    double receivedPowerW,
    double sampleWeight) const {

    if (!std::isfinite(rangeM) ||
        rangeM < 0.0 ||
        rangeM > config_.maxRangeM) {
        return;
    }

    const double contributionW =
        receivedPowerW * sampleWeight;

    if (!std::isfinite(contributionW) ||
        contributionW <= 0.0) {
        return;
    }

    /*
     * This is the ideal centre bin before applying
     * the range point-spread function.
     */
    const std::ptrdiff_t centreBin =
        static_cast<std::ptrdiff_t>(
            std::floor(
                rangeM /
                config_.rangeBinSizeM));

    /*
     * Calculate the valid kernel weight.
     *
     * This matters only near range zero or maximum range,
     * where part of the PSF would otherwise fall outside
     * the profile.
     */
    double validWeightSum = 0.0;

    for (std::size_t i = 0; i < RANGE_PSF.size(); ++i) {
        const std::ptrdiff_t offset =
            static_cast<std::ptrdiff_t>(i) -
            RANGE_PSF_CENTRE;

        const std::ptrdiff_t targetBin =
            centreBin + offset;

        if (targetBin < 0) {
            continue;
        }

        const std::size_t outputIndex =
            static_cast<std::size_t>(targetBin);

        if (outputIndex >= profile.bins.size()) {
            continue;
        }

        validWeightSum += RANGE_PSF[i];
    }

    if (validWeightSum <= 0.0) {
        return;
    }

    /*
     * Sparse range-PSF convolution:
     *
     * Replace the ideal one-bin reflection with a shifted,
     * scaled copy of the range PSF.
     */
    for (std::size_t i = 0; i < RANGE_PSF.size(); ++i) {
        const std::ptrdiff_t offset =
            static_cast<std::ptrdiff_t>(i) -
            RANGE_PSF_CENTRE;

        const std::ptrdiff_t targetBin =
            centreBin + offset;

        if (targetBin < 0) {
            continue;
        }

        const std::size_t outputIndex =
            static_cast<std::size_t>(targetBin);

        if (outputIndex >= profile.bins.size()) {
            continue;
        }

        const double normalizedWeight =
            RANGE_PSF[i] / validWeightSum;

        profile.bins[outputIndex] +=
            contributionW * normalizedWeight;
    }
}

} // namespace fmcw_radar
