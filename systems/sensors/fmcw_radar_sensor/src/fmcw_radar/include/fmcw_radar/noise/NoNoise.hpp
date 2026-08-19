#pragma once

#include "fmcw_radar/noise/INoiseModel.hpp"

namespace fmcw_radar {

class NoNoise final : public INoiseModel {
  public:
    void apply(AzimuthProfile& profile, const NoiseContext& context) const override;
};

} // namespace fmcw_radar
