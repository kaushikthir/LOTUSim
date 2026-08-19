#pragma once

namespace fmcw_radar {

class IAntennaPattern {
  public:
    virtual ~IAntennaPattern() = default;

    virtual double powerGain(double azimuthOffsetRad, double elevationOffsetRad) const = 0;
};

} // namespace fmcw_radar
