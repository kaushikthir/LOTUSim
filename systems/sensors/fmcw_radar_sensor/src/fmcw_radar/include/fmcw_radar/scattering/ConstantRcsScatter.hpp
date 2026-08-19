#pragma once

#include "fmcw_radar/scattering/IScatterModel.hpp"

namespace fmcw_radar {

class ConstantRcsScatter final : public IScatterModel {
  public:
    explicit ConstantRcsScatter(double rcsM2);

    double radarCrossSectionM2(const SurfaceHit& hit, const Vec3& incidentDirection, const Vec3& returnDirection, double frequencyHz) const override;

  private:
    double rcsM2_;
};

} // namespace fmcw_radar
