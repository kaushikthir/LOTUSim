#include <fmcw_radar/noise/NoNoise.hpp>

namespace fmcw_radar {

void NoNoise::apply(AzimuthProfile& profile, const NoiseContext& context) const {
    (void)profile;
    (void)context;

    // Ideal baseline:
    // intentionally no added noise.
}

} // namespace fmcw_radar
