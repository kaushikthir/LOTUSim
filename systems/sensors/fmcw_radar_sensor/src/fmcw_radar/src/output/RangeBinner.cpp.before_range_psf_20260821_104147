#include <fmcw_radar/output/RangeBinner.hpp>

#include <cmath>

namespace fmcw_radar {

RangeBinner::RangeBinner(const FmcwRadarConfig& config) : config_(config) {
    binCount_ = static_cast<std::size_t>(std::ceil(config_.maxRangeM / config_.rangeBinSizeM));
}

AzimuthProfile RangeBinner::makeProfile(double azimuthRad, double timestampSeconds) const {
    AzimuthProfile profile;

    profile.azimuthRad = azimuthRad;

    profile.timestampSeconds = timestampSeconds;

    profile.bins.assign(binCount_, 0.0);

    return profile;
}

void RangeBinner::add(AzimuthProfile& profile, double rangeM, double receivedPowerW, double sampleWeight) const {
    if (rangeM < 0.0 || rangeM > config_.maxRangeM) {
        return;
    }

    const std::size_t bin = static_cast<std::size_t>(std::floor(rangeM / config_.rangeBinSizeM));

    if (bin < profile.bins.size()) {
        profile.bins[bin] += receivedPowerW * sampleWeight;
    }
}

} // namespace fmcw_radar
