#include "fmcw_radar_sensor/fmcw_radar_sensor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>

#include <sdf/Element.hh>

#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "fmcw_radar/FmcwRadarConfig.hpp"

namespace {

std::string SanitizeRosToken(std::string value)
{
    std::string output;
    output.reserve(value.size());

    for (const unsigned char character : value) {
        if (std::isalnum(character) || character == '_') {
            output.push_back(
                static_cast<char>(
                    std::tolower(character)));
        }
        else if (output.empty() || output.back() != '_') {
            output.push_back('_');
        }
    }

    while (!output.empty() && output.front() == '_') {
        output.erase(output.begin());
    }

    while (!output.empty() && output.back() == '_') {
        output.pop_back();
    }

    if (output.empty()) {
        return "unnamed";
    }

    if (std::isdigit(
            static_cast<unsigned char>(output.front())))
    {
        output.insert(output.begin(), 'r');
        output.insert(output.begin() + 1, '_');
    }

    return output;
}

std::string VesselNameFromParent(
    const std::string& parent_name)
{
    std::vector<std::string> parts;
    std::string current;

    for (std::size_t index = 0;
         index < parent_name.size();
         ++index)
    {
        const bool double_colon =
            index + 1 < parent_name.size() &&
            parent_name[index] == ':' &&
            parent_name[index + 1] == ':';

        const bool separator =
            double_colon ||
            parent_name[index] == '/';

        if (separator) {
            if (!current.empty()) {
                parts.push_back(current);
                current.clear();
            }

            if (double_colon) {
                ++index;
            }

            continue;
        }

        current.push_back(parent_name[index]);
    }

    if (!current.empty()) {
        parts.push_back(current);
    }

    for (const std::string& part : parts) {
        const std::string clean = SanitizeRosToken(part);

        if (!clean.empty() &&
            clean != "world" &&
            clean != "lotusim") {
            return clean;
        }
    }

    return "vessel";
}

std::string BuildRadarTopic(
    const std::string& parent_name,
    const std::string& sensor_name,
    const std::string& suffix)
{
    return
        "/" +
        VesselNameFromParent(parent_name) +
        "/" +
        SanitizeRosToken(sensor_name) +
        "/" +
        SanitizeRosToken(suffix);
}


constexpr double kIntersectionEpsilon = 1.0e-8;
constexpr std::size_t kBvhLeafSize = 8;

double AxisValue(
    const gz::math::Vector3d& value,
    int axis)
{
    if (axis == 0) {
        return value.X();
    }

    if (axis == 1) {
        return value.Y();
    }

    return value.Z();
}

gz::math::Vector3d ComponentMinimum(
    const gz::math::Vector3d& a,
    const gz::math::Vector3d& b)
{
    return {
        std::min(a.X(), b.X()),
        std::min(a.Y(), b.Y()),
        std::min(a.Z(), b.Z())
    };
}

gz::math::Vector3d ComponentMaximum(
    const gz::math::Vector3d& a,
    const gz::math::Vector3d& b)
{
    return {
        std::max(a.X(), b.X()),
        std::max(a.Y(), b.Y()),
        std::max(a.Z(), b.Z())
    };
}

template<typename T>
T ReadOrDefault(
    const sdf::ElementPtr& parent,
    const std::string& tag,
    const T& default_value)
{
    if (!parent) {
        return default_value;
    }

    const auto element = parent->FindElement(tag);

    if (!element) {
        return default_value;
    }

    return element->Get<T>();
}

std::size_t ReadSizeOrDefault(
    const sdf::ElementPtr& parent,
    const std::string& tag,
    std::size_t default_value)
{
    const auto value = ReadOrDefault<unsigned int>(
        parent,
        tag,
        static_cast<unsigned int>(default_value));

    return static_cast<std::size_t>(value);
}

std::uint64_t ReadSeedOrDefault(
    const sdf::ElementPtr& parent,
    const std::string& tag,
    std::uint64_t default_value)
{
    const auto value = ReadOrDefault<unsigned int>(
        parent,
        tag,
        static_cast<unsigned int>(default_value));

    return static_cast<std::uint64_t>(value);
}

}  // namespace

namespace lotusim::sensor {

FmcwRadarSensor::FmcwRadarSensor(
    std::shared_ptr<spdlog::logger> logger,
    rclcpp::Node::SharedPtr node,
    const gz::sim::Entity& vessel_entity,
    const gz::sim::Entity& sensor_entity,
    const std::string& parent_name,
    const std::string& sensor_name)
    : ExteroceptiveSensor(
          logger,
          node,
          vessel_entity,
          sensor_entity,
          parent_name,
          sensor_name),
      m_node(node),
      m_ppi_topic(BuildRadarTopic(parent_name, sensor_name, "ppi")),
      m_returns_topic(BuildRadarTopic(parent_name, sensor_name, "returns")),
      m_vessel_entity_id(vessel_entity),
      m_sensor_entity_id(sensor_entity)
{
}

bool FmcwRadarSensor::CustomSensorLoad(
    const sdf::Sensor& sensor_sdf)
{
    try {
        fmcw_radar::FmcwRadarConfig config;

        config.name = m_sensor_name;

        if (sensor_sdf.UpdateRate() > 0.0) {
            config.rotationRateHz = sensor_sdf.UpdateRate();
        }

        const auto sensor_element = sensor_sdf.Element();

        if (!sensor_element) {
            m_logger->error(
                "FmcwRadarSensor [{}]: unable to access sensor SDF",
                m_sensor_name);

            return false;
        }

        const auto fmcw_element =
            sensor_element->FindElement("fmcw");

        if (fmcw_element) {
            config.carrierFrequencyHz =
                ReadOrDefault<double>(
                    fmcw_element,
                    "carrier_frequency_hz",
                    config.carrierFrequencyHz);

            config.maxRangeM =
                ReadOrDefault<double>(
                    fmcw_element,
                    "max_range_m",
                    config.maxRangeM);

            config.rangeBinSizeM =
                ReadOrDefault<double>(
                    fmcw_element,
                    "range_bin_size_m",
                    config.rangeBinSizeM);

            config.azimuthCount =
                ReadSizeOrDefault(
                    fmcw_element,
                    "azimuth_count",
                    config.azimuthCount);

            config.antennaModel =
                ReadOrDefault<std::string>(
                    fmcw_element,
                    "antenna_model",
                    config.antennaModel);

            config.horizontalBeamwidthDeg =
                ReadOrDefault<double>(
                    fmcw_element,
                    "horizontal_beamwidth_deg",
                    config.horizontalBeamwidthDeg);

            config.verticalBeamwidthDeg =
                ReadOrDefault<double>(
                    fmcw_element,
                    "vertical_beamwidth_deg",
                    config.verticalBeamwidthDeg);

            config.boresightGainLinear =
                ReadOrDefault<double>(
                    fmcw_element,
                    "boresight_gain_linear",
                    config.boresightGainLinear);

            config.transmitPowerW =
                ReadOrDefault<double>(
                    fmcw_element,
                    "transmit_power_w",
                    config.transmitPowerW);

            config.scatteringModel =
                ReadOrDefault<std::string>(
                    fmcw_element,
                    "scattering_model",
                    config.scatteringModel);

            config.constantRcsM2 =
                ReadOrDefault<double>(
                    fmcw_element,
                    "constant_rcs_m2",
                    config.constantRcsM2);

            config.horizontalSamples =
                ReadSizeOrDefault(
                    fmcw_element,
                    "horizontal_samples",
                    config.horizontalSamples);

            config.verticalSamples =
                ReadSizeOrDefault(
                    fmcw_element,
                    "vertical_samples",
                    config.verticalSamples);

            config.horizontalSampleSpanDeg =
                ReadOrDefault<double>(
                    fmcw_element,
                    "horizontal_sample_span_deg",
                    config.horizontalSampleSpanDeg);

            config.verticalSampleSpanDeg =
                ReadOrDefault<double>(
                    fmcw_element,
                    "vertical_sample_span_deg",
                    config.verticalSampleSpanDeg);

            config.noise.model =
                ReadOrDefault<std::string>(
                    fmcw_element,
                    "noise_model",
                    config.noise.model);

            config.noise.seed =
                ReadSeedOrDefault(
                    fmcw_element,
                    "noise_seed",
                    config.noise.seed);
        }

        m_max_range_m = config.maxRangeM;

        LoadExteroceptiveParams(sensor_sdf);

        m_radar_model =
            std::make_unique<fmcw_radar::FmcwRadar>(config);

        const auto& loaded = m_radar_model->config();

        m_logger->info(
            "FmcwRadarSensor [{}]: radar library created",
            m_sensor_name);

        m_logger->info(
            "FmcwRadarSensor [{}]: frequency={} Hz, range={} m, "
            "bin={} m, azimuths={}, rotation={} Hz",
            m_sensor_name,
            loaded.carrierFrequencyHz,
            loaded.maxRangeM,
            loaded.rangeBinSizeM,
            loaded.azimuthCount,
            loaded.rotationRateHz);

        m_logger->info(
            "FmcwRadarSensor [{}]: antenna={}, beam={} x {} deg, "
            "scattering={}, RCS={} m^2",
            m_sensor_name,
            loaded.antennaModel,
            loaded.horizontalBeamwidthDeg,
            loaded.verticalBeamwidthDeg,
            loaded.scatteringModel,
            loaded.constantRcsM2);

        if (!m_node) {
            throw std::runtime_error(
                "ROS 2 node is unavailable for the PPI publisher");
        }

        m_ppi_publisher =
            m_node->create_publisher<sensor_msgs::msg::Image>(
                m_ppi_topic,
                rclcpp::SensorDataQoS());


        m_returns_publisher =
            m_node->create_publisher<std_msgs::msg::Float32MultiArray>(
                m_returns_topic,
                rclcpp::SensorDataQoS());

        m_logger->info(
            "FmcwRadarSensor [{}]: publishing radar topics: "
            "PPI=[{}], returns=[{}]",
            m_sensor_name,
            m_ppi_topic,
            m_returns_topic);

        m_logger->info(
            "FmcwRadarSensor [{}]: publishing PPI images on "
            "[/fmcw_radar/ppi]",
            m_sensor_name);

        return true;
    } catch (const std::exception& exception) {
        m_radar_model.reset();

        m_logger->error(
            "FmcwRadarSensor [{}]: failed to create radar: {}",
            m_sensor_name,
            exception.what());

        return false;
    }
}

void FmcwRadarSensor::RebuildGeometryCache(
    const gz::sim::EntityComponentManager& ecm)
{
    m_triangles.clear();
    m_bvh_nodes.clear();
    m_bvh_root = -1;

    const gz::math::Pose3d sensor_world_pose =
        gz::sim::worldPose(m_sensor_entity_id, ecm);

    std::size_t collision_count = 0;
    std::size_t mesh_count = 0;
    std::size_t box_count = 0;
    std::size_t unsupported_count = 0;

    const auto to_radar_local =
        [&sensor_world_pose](
            const gz::math::Pose3d& collision_world_pose,
            const gz::math::Vector3d& local_point)
        {
            const gz::math::Vector3d world_point =
                collision_world_pose.Pos() +
                collision_world_pose.Rot().RotateVector(local_point);

            const gz::math::Vector3d gazebo_local =
                sensor_world_pose.Rot().RotateVectorReverse(
                    world_point - sensor_world_pose.Pos());

            /*
             * Convert Gazebo sensor-local coordinates:
             *
             *   +X forward
             *   +Y left
             *   +Z up
             *
             * into the FMCW radar coordinates:
             *
             *   +X right
             *   +Y up
             *   -Z forward
             */
            return gz::math::Vector3d(
                -gazebo_local.Y(),
                 gazebo_local.Z(),
                -gazebo_local.X());
        };

    const auto add_triangle =
        [this](
            const gz::math::Vector3d& a,
            const gz::math::Vector3d& b,
            const gz::math::Vector3d& c,
            std::int32_t object_id)
        {
            gz::math::Vector3d normal =
                (b - a).Cross(c - a);

            const double normal_length =
                normal.Length();

            if (normal_length <= kIntersectionEpsilon) {
                return;
            }

            normal = normal / normal_length;

            CachedTriangle triangle;
            triangle.a = a;
            triangle.b = b;
            triangle.c = c;
            triangle.normal = normal;

            triangle.boundsMin =
                ComponentMinimum(a, ComponentMinimum(b, c));

            triangle.boundsMax =
                ComponentMaximum(a, ComponentMaximum(b, c));

            triangle.centroid = {
                (a.X() + b.X() + c.X()) / 3.0,
                (a.Y() + b.Y() + c.Y()) / 3.0,
                (a.Z() + b.Z() + c.Z()) / 3.0
            };

            triangle.objectId = object_id;

            m_triangles.push_back(triangle);
        };

    ecm.Each<
        gz::sim::components::Collision,
        gz::sim::components::Geometry>(
        [&](const gz::sim::Entity& collision_entity,
            const gz::sim::components::Collision*,
            const gz::sim::components::Geometry* geometry_component)
            -> bool
        {
            const gz::sim::Entity owner_model =
                gz::sim::topLevelModel(collision_entity, ecm);

            /*
             * ExteroceptiveSensor has already selected nearby
             * top-level models for this measurement.
             */
            if (!IsModelInScope(owner_model)) {
                return true;
            }

            ++collision_count;

            const std::int32_t object_id =
                static_cast<std::int32_t>(collision_entity);

            const gz::math::Pose3d collision_world_pose =
                gz::sim::worldPose(collision_entity, ecm);

            const sdf::Geometry& geometry =
                geometry_component->Data();

            if (const sdf::Mesh* mesh_sdf = geometry.MeshShape()) {
                ++mesh_count;

                const gz::common::Mesh* mesh =
                    gz::sim::loadMesh(*mesh_sdf);

                if (!mesh) {
                    m_logger->warn(
                        "FmcwRadarSensor [{}]: unable to load mesh [{}]",
                        m_sensor_name,
                        mesh_sdf->Uri());

                    return true;
                }

                const gz::math::Vector3d scale =
                    mesh_sdf->Scale();

                for (unsigned int submesh_index = 0;
                     submesh_index < mesh->SubMeshCount();
                     ++submesh_index)
                {
                    const auto submesh =
                        mesh->SubMeshByIndex(submesh_index).lock();

                    if (!submesh) {
                        continue;
                    }

                    if (submesh->SubMeshPrimitiveType() !=
                        gz::common::SubMesh::TRIANGLES)
                    {
                        continue;
                    }

                    for (unsigned int index = 0;
                         index + 2 < submesh->IndexCount();
                         index += 3)
                    {
                        const int index_a =
                            submesh->Index(index);

                        const int index_b =
                            submesh->Index(index + 1);

                        const int index_c =
                            submesh->Index(index + 2);

                        if (index_a < 0 ||
                            index_b < 0 ||
                            index_c < 0)
                        {
                            continue;
                        }

                        const auto scale_vertex =
                            [&scale](
                                const gz::math::Vector3d& vertex)
                            {
                                return gz::math::Vector3d(
                                    vertex.X() * scale.X(),
                                    vertex.Y() * scale.Y(),
                                    vertex.Z() * scale.Z());
                            };

                        const gz::math::Vector3d local_a =
                            scale_vertex(
                                submesh->Vertex(
                                    static_cast<unsigned int>(index_a)));

                        const gz::math::Vector3d local_b =
                            scale_vertex(
                                submesh->Vertex(
                                    static_cast<unsigned int>(index_b)));

                        const gz::math::Vector3d local_c =
                            scale_vertex(
                                submesh->Vertex(
                                    static_cast<unsigned int>(index_c)));

                        add_triangle(
                            to_radar_local(
                                collision_world_pose,
                                local_a),
                            to_radar_local(
                                collision_world_pose,
                                local_b),
                            to_radar_local(
                                collision_world_pose,
                                local_c),
                            object_id);
                    }
                }

                return true;
            }

            if (const sdf::Box* box = geometry.BoxShape()) {
                ++box_count;

                const gz::math::Vector3d half =
                    box->Size() * 0.5;

                const gz::math::Vector3d local_vertices[8] = {
                    {-half.X(), -half.Y(), -half.Z()},
                    { half.X(), -half.Y(), -half.Z()},
                    { half.X(),  half.Y(), -half.Z()},
                    {-half.X(),  half.Y(), -half.Z()},
                    {-half.X(), -half.Y(),  half.Z()},
                    { half.X(), -half.Y(),  half.Z()},
                    { half.X(),  half.Y(),  half.Z()},
                    {-half.X(),  half.Y(),  half.Z()}
                };

                gz::math::Vector3d vertices[8];

                for (std::size_t i = 0; i < 8; ++i) {
                    vertices[i] =
                        to_radar_local(
                            collision_world_pose,
                            local_vertices[i]);
                }

                const int faces[12][3] = {
                    {0, 2, 1}, {0, 3, 2},
                    {4, 5, 6}, {4, 6, 7},
                    {0, 1, 5}, {0, 5, 4},
                    {1, 2, 6}, {1, 6, 5},
                    {2, 3, 7}, {2, 7, 6},
                    {3, 0, 4}, {3, 4, 7}
                };

                for (const auto& face : faces) {
                    add_triangle(
                        vertices[face[0]],
                        vertices[face[1]],
                        vertices[face[2]],
                        object_id);
                }

                return true;
            }

            ++unsupported_count;
            return true;
        });

    m_collision_count = collision_count;

    /*
     * These counts are still collected internally, but they are no
     * longer printed on every scan.
     */
    (void)mesh_count;
    (void)box_count;
    (void)unsupported_count;

    if (!m_triangles.empty()) {
        m_bvh_nodes.reserve(m_triangles.size() * 2);
        m_bvh_root =
            BuildBvhNode(0, m_triangles.size());
    }


}

int FmcwRadarSensor::BuildBvhNode(
    std::size_t start,
    std::size_t count)
{
    BvhNode node;
    node.start = start;
    node.count = count;

    node.boundsMin = {
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    };

    node.boundsMax = {
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()
    };

    gz::math::Vector3d centroid_min =
        node.boundsMin;

    gz::math::Vector3d centroid_max =
        node.boundsMax;

    for (std::size_t i = start;
         i < start + count;
         ++i)
    {
        node.boundsMin =
            ComponentMinimum(
                node.boundsMin,
                m_triangles[i].boundsMin);

        node.boundsMax =
            ComponentMaximum(
                node.boundsMax,
                m_triangles[i].boundsMax);

        centroid_min =
            ComponentMinimum(
                centroid_min,
                m_triangles[i].centroid);

        centroid_max =
            ComponentMaximum(
                centroid_max,
                m_triangles[i].centroid);
    }

    const int node_index =
        static_cast<int>(m_bvh_nodes.size());

    m_bvh_nodes.push_back(node);

    if (count <= kBvhLeafSize) {
        return node_index;
    }

    const gz::math::Vector3d extent =
        centroid_max - centroid_min;

    int split_axis = 0;

    if (extent.Y() > extent.X()) {
        split_axis = 1;
    }

    if (extent.Z() >
        AxisValue(extent, split_axis))
    {
        split_axis = 2;
    }

    if (AxisValue(extent, split_axis) <=
        kIntersectionEpsilon)
    {
        return node_index;
    }

    const std::size_t middle =
        start + count / 2;

    std::nth_element(
        m_triangles.begin() +
            static_cast<std::ptrdiff_t>(start),
        m_triangles.begin() +
            static_cast<std::ptrdiff_t>(middle),
        m_triangles.begin() +
            static_cast<std::ptrdiff_t>(start + count),
        [split_axis](
            const CachedTriangle& a,
            const CachedTriangle& b)
        {
            return AxisValue(a.centroid, split_axis) <
                   AxisValue(b.centroid, split_axis);
        });

    const int left =
        BuildBvhNode(
            start,
            middle - start);

    const int right =
        BuildBvhNode(
            middle,
            start + count - middle);

    m_bvh_nodes[node_index].left = left;
    m_bvh_nodes[node_index].right = right;
    m_bvh_nodes[node_index].count = 0;

    return node_index;
}

bool FmcwRadarSensor::RayIntersectsAabb(
    const gz::math::Vector3d& origin,
    const gz::math::Vector3d& direction,
    const gz::math::Vector3d& boundsMin,
    const gz::math::Vector3d& boundsMax,
    double maximumDistance)
{
    double minimum_distance = 0.0;
    double maximum_distance = maximumDistance;

    for (int axis = 0; axis < 3; ++axis) {
        const double ray_origin =
            AxisValue(origin, axis);

        const double ray_direction =
            AxisValue(direction, axis);

        const double slab_minimum =
            AxisValue(boundsMin, axis);

        const double slab_maximum =
            AxisValue(boundsMax, axis);

        if (std::abs(ray_direction) <=
            kIntersectionEpsilon)
        {
            if (ray_origin < slab_minimum ||
                ray_origin > slab_maximum)
            {
                return false;
            }

            continue;
        }

        const double inverse_direction =
            1.0 / ray_direction;

        double near_distance =
            (slab_minimum - ray_origin) *
            inverse_direction;

        double far_distance =
            (slab_maximum - ray_origin) *
            inverse_direction;

        if (near_distance > far_distance) {
            std::swap(
                near_distance,
                far_distance);
        }

        minimum_distance =
            std::max(
                minimum_distance,
                near_distance);

        maximum_distance =
            std::min(
                maximum_distance,
                far_distance);

        if (maximum_distance <
            minimum_distance)
        {
            return false;
        }
    }

    return true;
}

bool FmcwRadarSensor::RayIntersectsTriangle(
    const gz::math::Vector3d& origin,
    const gz::math::Vector3d& direction,
    const CachedTriangle& triangle,
    double& distance)
{
    const gz::math::Vector3d edge_1 =
        triangle.b - triangle.a;

    const gz::math::Vector3d edge_2 =
        triangle.c - triangle.a;

    const gz::math::Vector3d cross =
        direction.Cross(edge_2);

    const double determinant =
        edge_1.Dot(cross);

    if (std::abs(determinant) <=
        kIntersectionEpsilon)
    {
        return false;
    }

    const double inverse_determinant =
        1.0 / determinant;

    const gz::math::Vector3d from_a =
        origin - triangle.a;

    const double u =
        inverse_determinant *
        from_a.Dot(cross);

    if (u < 0.0 || u > 1.0) {
        return false;
    }

    const gz::math::Vector3d q =
        from_a.Cross(edge_1);

    const double v =
        inverse_determinant *
        direction.Dot(q);

    if (v < 0.0 || (u + v) > 1.0) {
        return false;
    }

    distance =
        inverse_determinant *
        edge_2.Dot(q);

    return distance >
           kIntersectionEpsilon;
}

bool FmcwRadarSensor::TraceOne(
    const fmcw_radar::Ray& ray,
    fmcw_radar::SurfaceHit& hit) const
{
    if (m_bvh_root < 0) {
        return false;
    }

    gz::math::Vector3d origin(
        ray.origin.x,
        ray.origin.y,
        ray.origin.z);

    gz::math::Vector3d direction(
        ray.direction.x,
        ray.direction.y,
        ray.direction.z);

    const double direction_length =
        direction.Length();

    if (direction_length <=
        kIntersectionEpsilon)
    {
        return false;
    }

    direction =
        direction / direction_length;

    double maximum_distance =
        m_max_range_m;

    if (ray.maxRangeM > 0.0) {
        maximum_distance =
            std::min(
                maximum_distance,
                ray.maxRangeM);
    }

    double nearest_distance =
        maximum_distance;

    const CachedTriangle* nearest_triangle =
        nullptr;

    thread_local std::vector<int> stack;
    stack.clear();

    if (stack.capacity() < 128) {
        stack.reserve(128);
    }

    stack.push_back(m_bvh_root);

    while (!stack.empty()) {
        const int node_index =
            stack.back();

        stack.pop_back();

        const BvhNode& node =
            m_bvh_nodes[
                static_cast<std::size_t>(
                    node_index)];

        if (!RayIntersectsAabb(
                origin,
                direction,
                node.boundsMin,
                node.boundsMax,
                nearest_distance))
        {
            continue;
        }

        if (node.IsLeaf()) {
            for (std::size_t i = node.start;
                 i < node.start + node.count;
                 ++i)
            {
                double candidate_distance = 0.0;

                if (!RayIntersectsTriangle(
                        origin,
                        direction,
                        m_triangles[i],
                        candidate_distance))
                {
                    continue;
                }

                if (candidate_distance <
                        nearest_distance &&
                    candidate_distance <=
                        maximum_distance)
                {
                    nearest_distance =
                        candidate_distance;

                    nearest_triangle =
                        &m_triangles[i];
                }
            }

            continue;
        }

        if (node.left >= 0) {
            stack.push_back(node.left);
        }

        if (node.right >= 0) {
            stack.push_back(node.right);
        }
    }

    if (!nearest_triangle) {
        return false;
    }

    gz::math::Vector3d hit_normal =
        nearest_triangle->normal;

    if (hit_normal.Dot(direction) > 0.0) {
        hit_normal =
            hit_normal * -1.0;
    }

    const gz::math::Vector3d hit_position =
        origin +
        direction * nearest_distance;

    hit = {};
    hit.distanceM = nearest_distance;

    hit.position = {
        hit_position.X(),
        hit_position.Y(),
        hit_position.Z()
    };

    hit.normal = {
        hit_normal.X(),
        hit_normal.Y(),
        hit_normal.Z()
    };

    hit.objectId =
        nearest_triangle->objectId;

    return true;
}

bool FmcwRadarSensor::traceNearest(
    const fmcw_radar::Ray& ray,
    fmcw_radar::SurfaceHit& hit) const
{
    return TraceOne(ray, hit);
}

std::vector<fmcw_radar::RayTraceResult>
FmcwRadarSensor::traceNearestBatch(
    const std::vector<fmcw_radar::Ray>& rays) const
{
    std::vector<fmcw_radar::RayTraceResult> results;
    results.resize(rays.size());

    if (rays.empty() || m_bvh_root < 0) {
        m_hits_this_scan = 0;
        return results;
    }

    std::size_t worker_count =
        static_cast<std::size_t>(
            std::thread::hardware_concurrency());

    if (worker_count == 0) {
        worker_count = 1;
    }

    worker_count =
        std::min(
            worker_count,
            rays.size());

    std::atomic<std::size_t> hit_count{0};

    const auto process_range =
        [this, &rays, &results, &hit_count](
            std::size_t begin,
            std::size_t end)
        {
            std::size_t local_hits = 0;

            for (std::size_t i = begin;
                 i < end;
                 ++i)
            {
                results[i].hit =
                    TraceOne(
                        rays[i],
                        results[i].surfaceHit);

                if (results[i].hit) {
                    ++local_hits;
                }
            }

            hit_count.fetch_add(
                local_hits,
                std::memory_order_relaxed);
        };

    if (worker_count == 1) {
        process_range(0, rays.size());
    } else {
        std::vector<std::thread> workers;
        workers.reserve(worker_count);

        const std::size_t chunk_size =
            (rays.size() +
             worker_count - 1) /
            worker_count;

        for (std::size_t worker = 0;
             worker < worker_count;
             ++worker)
        {
            const std::size_t begin =
                worker * chunk_size;

            const std::size_t end =
                std::min(
                    begin + chunk_size,
                    rays.size());

            if (begin >= end) {
                break;
            }

            workers.emplace_back(
                process_range,
                begin,
                end);
        }

        for (std::thread& worker : workers) {
            worker.join();
        }
    }

    m_hits_this_scan =
        hit_count.load(
            std::memory_order_relaxed);

    return results;
}


sensor_msgs::msg::Image FmcwRadarSensor::BuildPpiImage(
    const fmcw_radar::RadarFrame& frame) const
{
    sensor_msgs::msg::Image image;

    const std::size_t image_size =
        std::max<std::size_t>(
            64,
            m_ppi_image_size_px);

    image.header.frame_id = m_sensor_name;

    if (m_node) {
        image.header.stamp = m_node->now();
    }

    image.height =
        static_cast<std::uint32_t>(image_size);

    image.width =
        static_cast<std::uint32_t>(image_size);

    image.encoding =
        sensor_msgs::image_encodings::MONO8;

    image.is_bigendian = false;

    image.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(
            image_size);

    image.data.assign(
        image_size * image_size,
        std::uint8_t{0});

    if (!m_radar_model ||
        frame.profiles.empty())
    {
        return image;
    }

    const auto& config =
        m_radar_model->config();

    /*
     * Temporary zoomed display for nearby test objects.
     */
    const double ppi_display_range_m =
        std::min(
            100.0,
            config.maxRangeM);

    const double centre =
        0.5 *
        static_cast<double>(
            image_size - 1);

    const double display_radius =
        0.48 *
        static_cast<double>(
            image_size);

    /*
     * For every azimuth, find only the strongest detected bin
     * and draw one visible dot.
     */
    for (const auto& profile : frame.profiles) {
        double strongest_power = 0.0;
        std::size_t strongest_bin = 0;
        bool detection_found = false;

        for (std::size_t bin_index = 0;
             bin_index < profile.bins.size();
             ++bin_index)
        {
            const double power =
                profile.bins[bin_index];

            if (power > strongest_power &&
                std::isfinite(power))
            {
                strongest_power = power;
                strongest_bin = bin_index;
                detection_found = true;
            }
        }

        if (!detection_found ||
            strongest_power <= 0.0)
        {
            continue;
        }

        const double range_m =
            (static_cast<double>(strongest_bin) + 0.5) *
            config.rangeBinSizeM;

        if (range_m > ppi_display_range_m) {
            continue;
        }

        const double radius_px =
            (range_m / ppi_display_range_m) *
            display_radius;

        const int x =
            static_cast<int>(
                std::lround(
                    centre +
                    radius_px *
                    std::sin(profile.azimuthRad)));

        const int y =
            static_cast<int>(
                std::lround(
                    centre -
                    radius_px *
                    std::cos(profile.azimuthRad)));

        /*
         * Draw a 5 x 5 bright square so nearby detections
         * remain visible.
         */
        constexpr int dot_radius = 2;

        for (int dy = -dot_radius;
             dy <= dot_radius;
             ++dy)
        {
            for (int dx = -dot_radius;
                 dx <= dot_radius;
                 ++dx)
            {
                const int pixel_x = x + dx;
                const int pixel_y = y + dy;

                if (pixel_x < 0 ||
                    pixel_y < 0 ||
                    pixel_x >= static_cast<int>(image_size) ||
                    pixel_y >= static_cast<int>(image_size))
                {
                    continue;
                }

                const std::size_t pixel_index =
                    static_cast<std::size_t>(pixel_y) *
                        image_size +
                    static_cast<std::size_t>(pixel_x);

                image.data[pixel_index] = 255;
            }
        }
    }

    return image;
}

void FmcwRadarSensor::PublishReturns(
    const fmcw_radar::RadarFrame& frame)
{
    if (!m_returns_publisher ||
        !m_radar_model)
    {
        return;
    }

    const auto& config =
        m_radar_model->config();

    std_msgs::msg::Float32MultiArray message;

    /*
     * Packet header:
     *   0 = azimuth count
     *   1 = range-bin size in metres
     *   2 = maximum range in metres
     *   3 = rotation rate in hertz
     *
     * Each sparse return then contains:
     *   azimuth radians, range metres, power watts, azimuth index
     */
    message.data.reserve(4096);

    message.data.push_back(
        static_cast<float>(config.azimuthCount));

    message.data.push_back(
        static_cast<float>(config.rangeBinSizeM));

    message.data.push_back(
        static_cast<float>(config.maxRangeM));

    message.data.push_back(
        static_cast<float>(config.rotationRateHz));

    for (std::size_t azimuthIndex = 0;
         azimuthIndex < frame.profiles.size();
         ++azimuthIndex)
    {
        const auto& profile =
            frame.profiles[azimuthIndex];

        for (std::size_t binIndex = 0;
             binIndex < profile.bins.size();
             ++binIndex)
        {
            const double powerW =
                profile.bins[binIndex];

            if (!(powerW > 0.0) ||
                !std::isfinite(powerW))
            {
                continue;
            }

            const double rangeM =
                (static_cast<double>(binIndex) + 0.5) *
                config.rangeBinSizeM;

            message.data.push_back(
                static_cast<float>(profile.azimuthRad));

            message.data.push_back(
                static_cast<float>(rangeM));

            message.data.push_back(
                static_cast<float>(powerW));

            message.data.push_back(
                static_cast<float>(azimuthIndex));
        }
    }

    m_returns_publisher->publish(message);
}

void FmcwRadarSensor::PublishPpi(
    const fmcw_radar::RadarFrame& frame)
{
    if (!m_ppi_publisher) {
        return;
    }

    m_ppi_publisher->publish(BuildPpiImage(frame));
}

bool FmcwRadarSensor::UpdateSensor(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm)
{
    if (!BeginExteroceptiveMeasurement(
            info,
            ecm))
    {
        return false;
    }

    if (!m_radar_model) {
        m_logger->error(
            "FmcwRadarSensor [{}]: radar model is not loaded",
            m_sensor_name);

        return false;
    }

    const auto& config =
        m_radar_model->config();

    const std::size_t ray_count =
        config.azimuthCount *
        config.horizontalSamples *
        config.verticalSamples;

    RebuildGeometryCache(ecm);

    if (m_triangles.empty()) {
        if (!m_warned_empty_scene) {
            m_logger->warn(
                "FmcwRadarSensor [{}]: pocket scene has no target "
                "triangles. The radar platform is intentionally excluded.",
                m_sensor_name);

            m_warned_empty_scene = true;
        }
    } else {
        m_warned_empty_scene = false;
    }

    const fmcw_radar::RadarPose radar_pose{};

    const double timestamp_seconds =
        std::chrono::duration<double>(
            info.simTime).count();

    m_hits_this_scan = 0;

    const fmcw_radar::RadarFrame frame =
        m_radar_model->scan(
            *this,
            radar_pose,
            timestamp_seconds,
            nullptr);

    PublishPpi(frame);
    PublishReturns(frame);

    ++m_scan_number;

    m_logger->info(
        "FmcwRadarSensor [{}]: scan {} | "
        "collision_objects={} | radar_ray_hits={}",
        m_sensor_name,
        m_scan_number,
        m_collision_count,
        m_hits_this_scan);

    m_last_measurement_time = info.simTime;
    return true;
}

}  // namespace lotusim::sensor
