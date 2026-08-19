#include <fmcw_radar/response/RadarReturnModel.hpp>

#include <fmcw_radar/math/RadarMath.hpp>

namespace fmcw_radar {

RadarReturnModel::RadarReturnModel(const FmcwRadarConfig& config, const IPropagationModel& propagation, const IScatterModel& scattering)
    : config_(config),

      propagation_(propagation),

      scattering_(scattering) {
    const double wavelength = math::wavelengthM(config_.carrierFrequencyHz);

    /*
     * Effective receive aperture:
     *
     * Ae = G lambda^2 / (4 pi)
     */
    receiveApertureM2_ = config_.boresightGainLinear * wavelength * wavelength / (4.0 * math::PI);
}

double RadarReturnModel::calculate(const Ray& ray, const SurfaceHit& hit, double antennaGain) const {
    const double range = hit.distanceM;

    if (range <= 0.0 || range > config_.maxRangeM) {
        return 0.0;
    }

    /*
     * Outbound radar power density.
     */
    const double incidentPowerDensity = config_.transmitPowerW * config_.boresightGainLinear * antennaGain * propagation_.oneWayFactor(range, config_.carrierFrequencyHz);

    /*
     * Target scattering.
     */
    const double rcs = scattering_.radarCrossSectionM2(hit, ray.direction, -ray.direction, config_.carrierFrequencyHz);

    /*
     * Return propagation.
     */
    const double returnedPowerDensity = incidentPowerDensity * rcs * propagation_.oneWayFactor(range, config_.carrierFrequencyHz);

    /*
     * Receive antenna.
     */
    return returnedPowerDensity * receiveApertureM2_ * antennaGain;
}

} // namespace fmcw_radar
