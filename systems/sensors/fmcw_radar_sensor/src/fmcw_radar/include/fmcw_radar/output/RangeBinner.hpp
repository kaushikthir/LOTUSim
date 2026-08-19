#pragma once

#include "fmcw_radar/FmcwRadarConfig.hpp"
#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

class RangeBinner {
  public:
    explicit RangeBinner(const FmcwRadarConfig& config);

    AzimuthProfile makeProfile(double azimuthRad, double timestampSeconds) const;

    void add(AzimuthProfile& profile, double rangeM, double receivedPowerW, double sampleWeight) const;

  private:
    const FmcwRadarConfig& config_;

    std::size_t binCount_;
};

} // namespace fmcw_radar
