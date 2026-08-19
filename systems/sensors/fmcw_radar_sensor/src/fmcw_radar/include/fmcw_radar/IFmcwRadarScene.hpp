#pragma once

#include <vector>

#include "fmcw_radar/FmcwRadarPrimitives.hpp"

namespace fmcw_radar {

class IFmcwRadarScene {
  public:
    virtual ~IFmcwRadarScene() = default;

    /*
     * Query one ray.
     *
     * Kept for simple scene backends and backwards compatibility.
     */
    virtual bool traceNearest(
        const Ray& ray,
        SurfaceHit& hit) const = 0;

    /*
     * Query many rays at once.
     *
     * The default implementation calls traceNearest() repeatedly.
     * Accelerated CPU or GPU backends override this method.
     */
    virtual std::vector<RayTraceResult> traceNearestBatch(
        const std::vector<Ray>& rays) const {
        std::vector<RayTraceResult> results;
        results.resize(rays.size());

        for (std::size_t i = 0; i < rays.size(); ++i) {
            results[i].hit =
                traceNearest(rays[i], results[i].surfaceHit);
        }

        return results;
    }
};

} // namespace fmcw_radar
