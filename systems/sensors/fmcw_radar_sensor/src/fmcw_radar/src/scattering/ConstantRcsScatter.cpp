#include <algorithm>
#include <fmcw_radar/scattering/ConstantRcsScatter.hpp>

namespace fmcw_radar {

ConstantRcsScatter::ConstantRcsScatter(double rcsM2)
    : rcsM2_(std::max(0.0, rcsM2))
{
}

double ConstantRcsScatter::radarCrossSectionM2(
    const SurfaceHit& hit,
    const Vec3& incidentDirection,
    const Vec3& returnDirection,
    double frequencyHz) const
{
    
    (void)hit;
    (void)incidentDirection;
    (void)returnDirection;
    (void)frequencyHz;

    return rcsM2_;
}

}  // namespace fmcw_radar


/*
 * ============================================================
 * NOMINAL MARITIME RCS REFERENCE VALUES
 * ============================================================
 *
 * These values are intended as Version 1 engineering estimates
 * for the 77 GHz FMCW radar simulator.
 *
 * IMPORTANT:
 * Radar Cross Section (RCS) is not a fixed property of a vessel.
 * It varies with:
 *
 *   - radar frequency
 *   - aspect angle
 *   - elevation / grazing angle
 *   - vessel geometry
 *   - material
 *   - polarization
 *   - sea-surface interaction
 *
 * Therefore, the values below should be treated as nominal
 * class-level values rather than exact physical signatures.
 *
 * ------------------------------------------------------------
 * SMALL VESSEL ~10 m
 * ------------------------------------------------------------
 *
 * Published reference:
 *   2.5 m^2 at X-band
 *
 * Suggested 77 GHz V1 value:
 *   ~7 m^2
 *
 * Basis:
 *   Frequency-scaled engineering estimate from a marine-radar
 *   reference target.
 *
 * Source:
 *   IMO MSC.192(79) Radar Performance Standard
 *
 *   https://puc.overheid.nl/nsi/doc/PUC_1574_14/1/
 *
 *
 * ------------------------------------------------------------
 * SMALL YACHT / CABIN CRUISER ~12–13 m
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~10–13 m^2 median RCS at 94 GHz
 *
 * Suggested 77 GHz V1 value:
 *   ~12 m^2
 *
 * Basis:
 *   Closest direct W-band proxy currently available.
 *   Measurements include a ~41 ft flybridge cruiser.
 *
 * Source:
 *   G. Brooker, C. Lobsey, R. Hennessy
 *   "Radar Cross Sections of Small Boats at 94 GHz"
 *
 *   https://www.researchgate.net/publication/224361461_Radar_cross_sections_of_small_boats_at_94_GHZ
 *
 *
 * ------------------------------------------------------------
 * FISHING / WORK BOAT ~15–30 m
 * ------------------------------------------------------------
 *
 * Published references:
 *   8.5 m fishing vessel:
 *       ~2–10 m^2 depending on aspect
 *
 *   42 m coaster:
 *       ~20–680 m^2 depending on aspect
 *
 * Suggested 77 GHz V1 value:
 *   ~100 m^2
 *
 * Basis:
 *   Interpolated engineering value between published
 *   small-fishing-vessel and coaster measurements.
 *
 * Source:
 *   Kelvin Hughes KH3200 Marine Radar Documentation
 *
 *   https://www.scribd.com/document/1019722259/KH3200-iss-5
 *
 *
 * ------------------------------------------------------------
 * MILITARY / FAST PATROL BOAT
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~20–500 m^2 at approximately 5–10 GHz
 *
 * Suggested 77 GHz V1 value:
 *   ~320 m^2
 *
 * Basis:
 *   Frequency-scaled engineering estimate using the approximate
 *   ship-RCS frequency dependence sigma proportional to sqrt(f).
 *
 * Source:
 *   Georgia Tech / NTIA
 *   Radar Fundamentals
 *
 *   https://its.ntia.gov/media/31135/ShowmanRadarFundamentalsISART2011.pdf
 *
 *
 * ------------------------------------------------------------
 * SMALL COASTER ~42 m
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~20–680 m^2 depending strongly on aspect angle
 *
 * Suggested 77 GHz V1 value:
 *   ~330 m^2
 *
 * Basis:
 *   Representative engineering value based on the published
 *   aspect-dependent range.
 *
 * Source:
 *   Kelvin Hughes KH3200 Marine Radar Documentation
 *
 *   https://www.scribd.com/document/1019722259/KH3200-iss-5
 *
 *
 * ------------------------------------------------------------
 * MERCHANT SHIP >500 GT
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~1000 m^2 at S-band
 *   ~1800 m^2 at X-band
 *
 * Suggested 77 GHz V1 value:
 *   ~5000 m^2
 *
 * Basis:
 *   X-band to 77 GHz engineering extrapolation.
 *
 * Source:
 *   Kelvin Hughes KH3200 Marine Radar Documentation
 *
 *   https://www.scribd.com/document/1019722259/KH3200-iss-5
 *
 *
 * ------------------------------------------------------------
 * FRIGATE ~1–2 kton
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~5000–10000 m^2 at approximately 5–10 GHz
 *
 * Suggested 77 GHz V1 value:
 *   ~23000 m^2
 *
 * Basis:
 *   Frequency-scaled engineering estimate using the approximate
 *   ship-RCS relationship sigma proportional to sqrt(f).
 *
 * Source:
 *   Georgia Tech / NTIA
 *   Radar Fundamentals
 *
 *   https://its.ntia.gov/media/31135/ShowmanRadarFundamentalsISART2011.pdf
 *
 *
 * ------------------------------------------------------------
 * LARGE MERCHANT SHIP >5000 GT
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~30000 m^2 at S-band
 *   ~50000 m^2 at X-band
 *
 * Suggested 77 GHz V1 value:
 *   ~140000 m^2
 *
 * Basis:
 *   X-band to 77 GHz engineering extrapolation.
 *
 * Source:
 *   Kelvin Hughes KH3200 Marine Radar Documentation
 *
 *   https://www.scribd.com/document/1019722259/KH3200-iss-5
 *
 *
 * ------------------------------------------------------------
 * DESTROYER ~3–5 kton
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~30000–60000 m^2 at approximately 5–10 GHz
 *
 * Suggested 77 GHz V1 value:
 *   ~140000 m^2
 *
 * Basis:
 *   Frequency-scaled engineering estimate using the approximate
 *   ship-RCS relationship sigma proportional to sqrt(f).
 *
 * Source:
 *   Georgia Tech / NTIA
 *   Radar Fundamentals
 *
 *   https://its.ntia.gov/media/31135/ShowmanRadarFundamentalsISART2011.pdf
 *
 *
 * ------------------------------------------------------------
 * LARGE TANKER / CONTAINER SHIP
 * ------------------------------------------------------------
 *
 * Published reference:
 *   Large merchant vessel >5000 GT:
 *       ~50000 m^2 at X-band
 *
 * Suggested 77 GHz V1 value:
 *   ~150000–300000 m^2
 *
 * Basis:
 *   Engineering estimate based on large-merchant-vessel
 *   reference values and vessel size.
 *
 *   This is NOT a direct 77 GHz measurement.
 *
 * Source:
 *   Kelvin Hughes KH3200 Marine Radar Documentation
 *
 *   https://www.scribd.com/document/1019722259/KH3200-iss-5
 *
 *
 * ------------------------------------------------------------
 * CRUISER ~7–20 kton
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~100000–400000 m^2 at approximately 5–10 GHz
 *
 * Suggested 77 GHz V1 value:
 *   ~640000 m^2
 *
 * Basis:
 *   Frequency-scaled engineering estimate using the approximate
 *   ship-RCS relationship sigma proportional to sqrt(f).
 *
 * Source:
 *   Georgia Tech / NTIA
 *   Radar Fundamentals
 *
 *   https://its.ntia.gov/media/31135/ShowmanRadarFundamentalsISART2011.pdf
 *
 *
 * ------------------------------------------------------------
 * AIRCRAFT CARRIER ~20–40 kton
 * ------------------------------------------------------------
 *
 * Published reference:
 *   ~300000–1000000 m^2 at approximately 5–10 GHz
 *
 * Suggested 77 GHz V1 value:
 *   ~1750000 m^2
 *
 * Basis:
 *   Frequency-scaled engineering estimate using the approximate
 *   ship-RCS relationship sigma proportional to sqrt(f).
 *
 * Source:
 *   Georgia Tech / NTIA
 *   Radar Fundamentals
 *
 *   https://its.ntia.gov/media/31135/ShowmanRadarFundamentalsISART2011.pdf
 *
 *
 * ============================================================
 * IMPORTANT MODELLING NOTE
 * ============================================================
 *
 * Only the small-yacht / cabin-cruiser value above is based on
 * a directly measured W-band vessel RCS near the simulator
 * frequency.
 *
 * Most larger-vessel values are engineering estimates derived
 * from lower-frequency marine / naval RCS data.
 *
 * These values are suitable for an initial nominal-RCS model:
 *
 *     sigma = sigma(asset_class)
 *
 * A future model should ideally use:
 *
 *     sigma = sigma(
 *         asset_class,
 *         aspect_angle,
 *         frequency,
 *         polarization
 *     )
 *
 * ============================================================
 */