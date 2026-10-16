#pragma once

#include <cstddef>
#include <cstdint>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <memory>
#include <sdf/Geometry.hh>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "fmcw_radar/IFmcwRadarScene.hpp"
#include "fmcw_radar/fmcw_radar.hpp"
#include "lotusim_sensor_base/exteroceptive_sensor.hpp"

namespace lotusim::sensor {

class FmcwRadarSensor final : public ExteroceptiveSensor,
                              public fmcw_radar::IFmcwRadarScene {
public:
    FmcwRadarSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~FmcwRadarSensor() override = default;

    bool CustomSensorLoad(const sdf::Sensor& sdf) override;

    bool UpdateSensor(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm) override;

    bool traceNearest(const fmcw_radar::Ray& ray, fmcw_radar::SurfaceHit& hit)
        const override;

    std::vector<fmcw_radar::RayTraceResult> traceNearestBatch(
        const std::vector<fmcw_radar::Ray>& rays) const override;

private:
    /*
     * Per-collision data that does not change between scans: identity,
     * names, and the triangulated shape in the collision's own frame.
     * Built once per collision entity and reused every scan.
     */
    struct CollisionGeometry {
        std::int64_t objectId{-1};
        std::string objectName;

        std::int64_t modelId{-1};
        std::string modelName;

        std::int64_t collisionId{-1};
        std::string collisionName;

        std::int32_t materialId{-1};
        std::string materialName;

        // Three entries per triangle, scale already applied.
        std::vector<gz::math::Vector3d> localVertices;
    };

    struct CachedTriangle {
        gz::math::Vector3d a;
        gz::math::Vector3d b;
        gz::math::Vector3d c;

        gz::math::Vector3d normal;
        gz::math::Vector3d boundsMin;
        gz::math::Vector3d boundsMax;
        gz::math::Vector3d centroid;

        // Points into m_collision_cache, which is not modified during a scan.
        const CollisionGeometry* source{nullptr};
    };

    struct BvhNode {
        gz::math::Vector3d boundsMin;
        gz::math::Vector3d boundsMax;

        int left{-1};
        int right{-1};

        std::size_t start{0};
        std::size_t count{0};

        bool IsLeaf() const
        {
            return left < 0 && right < 0;
        }
    };

    void RebuildGeometryCache(const gz::sim::EntityComponentManager& ecm);

    const CollisionGeometry& CachedCollisionGeometry(
        gz::sim::Entity collision_entity,
        gz::sim::Entity owner_model,
        const sdf::Geometry& geometry,
        const gz::sim::EntityComponentManager& ecm);

    int BuildBvhNode(std::size_t start, std::size_t count);

    bool TraceOne(const fmcw_radar::Ray& ray, fmcw_radar::SurfaceHit& hit)
        const;

    static bool RayIntersectsAabb(
        const gz::math::Vector3d& origin,
        const gz::math::Vector3d& direction,
        const gz::math::Vector3d& boundsMin,
        const gz::math::Vector3d& boundsMax,
        double maximumDistance);

    static bool RayIntersectsTriangle(
        const gz::math::Vector3d& origin,
        const gz::math::Vector3d& direction,
        const CachedTriangle& triangle,
        double& distance);

    void PublishReturns(const fmcw_radar::RadarFrame& frame);

    rclcpp::Node::SharedPtr m_node;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        m_returns_publisher;

    std::string m_returns_topic;

    std::unique_ptr<fmcw_radar::FmcwRadar> m_radar_model;

    gz::sim::Entity m_sensor_entity_id;

    std::unordered_map<gz::sim::Entity, CollisionGeometry> m_collision_cache;

    std::vector<CachedTriangle> m_triangles;
    std::vector<BvhNode> m_bvh_nodes;
    int m_bvh_root{-1};

    double m_max_range_m{1000.0};

    std::size_t m_collision_count{0};

    mutable std::size_t m_hits_this_scan{0};
    std::size_t m_scan_number{0};

    bool m_warned_empty_scene{false};
};

}  // namespace lotusim::sensor
