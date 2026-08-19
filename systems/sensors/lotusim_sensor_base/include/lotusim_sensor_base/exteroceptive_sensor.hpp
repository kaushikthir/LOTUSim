#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/math/Pose3.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>

#include <sdf/Sensor.hh>

#include "lotusim_sensor_base/custom_sensor.hpp"

namespace lotusim::sensor {

/*
 * A world model selected by the exteroceptive broad-phase scope.
 *
 * This does not mean the radar or sonar detected the model.
 * It only means that the model is close enough to be considered.
 */
struct ScopedEntity
{
    gz::sim::Entity model_entity{
        gz::sim::kNullEntity};

    std::string name;

    gz::math::Pose3d world_pose;

    double distance_m{0.0};
};

/*
 * Base class for sensors that observe external world geometry,
 * such as radar and sonar.
 *
 * ExteroceptiveSensor:
 *   - controls the broad-phase world scope
 *   - excludes the sensor's own platform
 *   - provides the nearby model list
 *
 * It does not control:
 *   - radar ray length
 *   - sonar ray length
 *   - range bins
 *   - scattering
 *   - sensor physics
 */
class ExteroceptiveSensor : public CustomSensor
{
public:
    ExteroceptiveSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& platform_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~ExteroceptiveSensor() override;

protected:
    /*
     * Call this once at the beginning of the derived sensor update.
     *
     * It:
     *   1. checks the CustomSensor measurement timing
     *   2. updates the sensor world pose
     *   3. refreshes the exteroceptive broad-phase scope
     */
    bool BeginExteroceptiveMeasurement(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm);

    /*
     * Loads:
     *
     * <exteroceptive>
     *   <scope_radius_m>...</scope_radius_m>
     * </exteroceptive>
     *
     * This value is independent from radar or sonar range.
     */
    void LoadExteroceptiveParams(
        const sdf::Sensor& sensor_sdf);

    /*
     * Returns true when the top-level model was selected by the
     * current exteroceptive broad-phase scope.
     */
    bool IsModelInScope(
        gz::sim::Entity model_entity) const;

    double ScopeRadiusM() const noexcept;

    const std::vector<ScopedEntity>&
    ScopedEntities() const noexcept;

    const gz::math::Pose3d&
    SensorWorldPose() const noexcept;

    /*
     * Derived sensors may add extra candidate filtering.
     */
    virtual bool AcceptScopedEntity(
        gz::sim::Entity model_entity,
        const gz::sim::EntityComponentManager& ecm) const;

private:
    void RefreshScope(
        const gz::sim::EntityComponentManager& ecm);

    gz::sim::Entity m_platform_entity{
        gz::sim::kNullEntity};

    gz::sim::Entity m_sensor_entity{
        gz::sim::kNullEntity};

    double m_scope_radius_m{0.0};

    gz::math::Pose3d m_sensor_world_pose;

    std::vector<ScopedEntity> m_scoped_entities;

    std::unordered_set<gz::sim::Entity>
        m_scoped_model_lookup;
};

}  // namespace lotusim::sensor
