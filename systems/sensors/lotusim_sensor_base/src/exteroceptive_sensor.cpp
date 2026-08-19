#include "lotusim_sensor_base/exteroceptive_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <gz/sim/Util.hh>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <sdf/Element.hh>

namespace lotusim::sensor {

ExteroceptiveSensor::ExteroceptiveSensor(
    std::shared_ptr<spdlog::logger> logger,
    rclcpp::Node::SharedPtr node,
    const gz::sim::Entity& platform_entity,
    const gz::sim::Entity& sensor_entity,
    const std::string& parent_name,
    const std::string& sensor_name)
    : CustomSensor(
          std::move(logger),
          std::move(node),
          platform_entity,
          sensor_entity,
          parent_name,
          sensor_name),
      m_platform_entity(platform_entity),
      m_sensor_entity(sensor_entity)
{
}

ExteroceptiveSensor::~ExteroceptiveSensor() = default;

bool ExteroceptiveSensor::BeginExteroceptiveMeasurement(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm)
{
    /*
     * CustomSensor controls the measurement update rate.
     */
    if (!EnableMeasurement(info.simTime)) {
        return false;
    }

    m_sensor_world_pose =
        gz::sim::worldPose(
            m_sensor_entity,
            ecm);

    /*
     * Refresh the broad-phase candidates for this measurement.
     */
    RefreshScope(ecm);

    return true;
}

void ExteroceptiveSensor::LoadExteroceptiveParams(
    const sdf::Sensor& sensor_sdf)
{
    const sdf::ElementPtr sensor_element =
        sensor_sdf.Element();

    if (!sensor_element) {
        throw std::runtime_error(
            "ExteroceptiveSensor: sensor SDF element is missing.");
    }

    const sdf::ElementPtr exteroceptive_element =
        sensor_element->FindElement("exteroceptive");

    if (!exteroceptive_element) {
        throw std::runtime_error(
            "ExteroceptiveSensor: missing "
            "<exteroceptive> SDF element.");
    }

    const sdf::ElementPtr radius_element =
        exteroceptive_element->FindElement(
            "scope_radius_m");

    if (!radius_element) {
        throw std::runtime_error(
            "ExteroceptiveSensor: missing "
            "<scope_radius_m> inside <exteroceptive>.");
    }

    const double radius_m =
        radius_element->Get<double>();

    if (!std::isfinite(radius_m) ||
        radius_m <= 0.0)
    {
        throw std::runtime_error(
            "ExteroceptiveSensor: scope_radius_m "
            "must be greater than zero.");
    }

    m_scope_radius_m = radius_m;
}

bool ExteroceptiveSensor::IsModelInScope(
    gz::sim::Entity model_entity) const
{
    return
        m_scoped_model_lookup.find(model_entity) !=
        m_scoped_model_lookup.end();
}

double ExteroceptiveSensor::ScopeRadiusM() const noexcept
{
    return m_scope_radius_m;
}

const std::vector<ScopedEntity>&
ExteroceptiveSensor::ScopedEntities() const noexcept
{
    return m_scoped_entities;
}

const gz::math::Pose3d&
ExteroceptiveSensor::SensorWorldPose() const noexcept
{
    return m_sensor_world_pose;
}

bool ExteroceptiveSensor::AcceptScopedEntity(
    gz::sim::Entity,
    const gz::sim::EntityComponentManager&) const
{
    return true;
}

void ExteroceptiveSensor::RefreshScope(
    const gz::sim::EntityComponentManager& ecm)
{
    m_scoped_entities.clear();
    m_scoped_model_lookup.clear();

    std::unordered_set<gz::sim::Entity>
        visited_models;

    const gz::math::Vector3d sensor_position =
        m_sensor_world_pose.Pos();

    ecm.Each<gz::sim::components::Model>(
        [&](const gz::sim::Entity& candidate_entity,
            const gz::sim::components::Model*) -> bool
        {
            gz::sim::Entity model_entity =
                gz::sim::topLevelModel(
                    candidate_entity,
                    ecm);

            if (model_entity == gz::sim::kNullEntity) {
                model_entity = candidate_entity;
            }

            /*
             * Nested models may resolve to the same top-level model.
             */
            if (!visited_models.insert(model_entity).second) {
                return true;
            }

            /*
             * Do not include the vessel or platform carrying the
             * exteroceptive sensor.
             */
            if (model_entity == m_platform_entity) {
                return true;
            }

            const gz::math::Pose3d model_world_pose =
                gz::sim::worldPose(
                    model_entity,
                    ecm);

            const double distance_m =
                (
                    model_world_pose.Pos() -
                    sensor_position
                ).Length();

            /*
             * This is the broad-phase scope.
             *
             * It only determines whether the model may be passed
             * to the radar or sonar geometry system.
             */
            if (distance_m > m_scope_radius_m) {
                return true;
            }

            if (!AcceptScopedEntity(
                    model_entity,
                    ecm))
            {
                return true;
            }

            std::string model_name;

            if (const auto* name_component =
                    ecm.Component<
                        gz::sim::components::Name>(
                        model_entity))
            {
                model_name =
                    name_component->Data();
            }

            ScopedEntity scoped_entity;

            scoped_entity.model_entity =
                model_entity;

            scoped_entity.name =
                std::move(model_name);

            scoped_entity.world_pose =
                model_world_pose;

            scoped_entity.distance_m =
                distance_m;

            m_scoped_entities.push_back(
                std::move(scoped_entity));

            m_scoped_model_lookup.insert(
                model_entity);

            return true;
        });

    std::sort(
        m_scoped_entities.begin(),
        m_scoped_entities.end(),
        [](const ScopedEntity& left,
           const ScopedEntity& right)
        {
            return
                left.distance_m <
                right.distance_m;
        });
}

}  // namespace lotusim::sensor
