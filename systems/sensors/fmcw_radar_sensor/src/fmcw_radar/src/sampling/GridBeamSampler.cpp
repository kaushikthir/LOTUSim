#include <fmcw_radar/sampling/GridBeamSampler.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fmcw_radar {

GridBeamSampler::GridBeamSampler(double horizontalHalfSpanRad, double verticalHalfSpanRad,

                                 std::size_t horizontalSamples, std::size_t verticalSamples) {
    if (horizontalSamples == 0 || verticalSamples == 0) {
        throw std::runtime_error("Beam sample counts cannot be zero");
    }

    samples_.reserve(horizontalSamples * verticalSamples);

    double totalWeight = 0.0;

    for (std::size_t y = 0; y < verticalSamples; ++y) {
        const double fy = (static_cast<double>(y) + 0.5) / static_cast<double>(verticalSamples);

        const double elevation = -verticalHalfSpanRad + 2.0 * verticalHalfSpanRad * fy;

        for (std::size_t x = 0; x < horizontalSamples; ++x) {
            const double fx = (static_cast<double>(x) + 0.5) / static_cast<double>(horizontalSamples);

            const double azimuth = -horizontalHalfSpanRad + 2.0 * horizontalHalfSpanRad * fx;

            const double weight = std::max(0.0, std::cos(elevation));

            samples_.push_back({azimuth, elevation, weight});

            totalWeight += weight;
        }
    }

    if (totalWeight > 0.0) {
        for (BeamSample& sample : samples_) {
            sample.weight /= totalWeight;
        }
    }
}

const std::vector<BeamSample>& GridBeamSampler::samples() const {
    return samples_;
}

} // namespace fmcw_radar
