#pragma once

#include "fmcw_radar/antenna/IAntennaPattern.hpp"

namespace fmcw_radar {

class SincAntennaPattern final : public IAntennaPattern {
  public:
    SincAntennaPattern(double horizontalHpbwRad, double verticalHpbwRad);

    double powerGain(double azimuthOffsetRad, double elevationOffsetRad) const override;

  private:
    double horizontalHpbwRad_;
    double verticalHpbwRad_;
};

} // namespace fmcw_radar
