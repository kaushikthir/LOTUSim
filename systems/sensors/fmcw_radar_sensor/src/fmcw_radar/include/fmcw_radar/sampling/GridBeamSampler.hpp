#pragma once

#include <cstddef>
#include <vector>

#include "fmcw_radar/sampling/IBeamSampler.hpp"

namespace fmcw_radar {

class GridBeamSampler final : public IBeamSampler {
  public:
    GridBeamSampler(double horizontalHalfSpanRad, double verticalHalfSpanRad,

                    std::size_t horizontalSamples, std::size_t verticalSamples);

    const std::vector<BeamSample>& samples() const override;

  private:
    std::vector<BeamSample> samples_;
};

} // namespace fmcw_radar
