#pragma once

#include "fmcw_radar/propagation/IPropagationModel.hpp"

namespace fmcw_radar {

class FreeSpacePropagation final : public IPropagationModel {
  public:
    double oneWayFactor(double distanceM, double frequencyHz) const override;
};

} // namespace fmcw_radar
