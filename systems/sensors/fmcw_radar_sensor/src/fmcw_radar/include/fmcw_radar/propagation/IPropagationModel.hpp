#pragma once

namespace fmcw_radar {

class IPropagationModel {
  public:
    virtual ~IPropagationModel() = default;

    virtual double oneWayFactor(double distanceM, double frequencyHz) const = 0;
};

} // namespace fmcw_radar
