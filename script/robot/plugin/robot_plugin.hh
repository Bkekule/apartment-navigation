#pragma once

#include "logging.hh"

#include <gz/msgs/twist.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <mutex>
#include <string>

namespace boris_apartment {

/**
 * @brief Gazebo Sim plugin for differential drive robot via `cmd_vel`.
 *
 * Subscribes to `/model/<name>/cmd_vel` (gz.msgs.Twist) and uses differential
 * drive kinematics to map to left and right wheel angular velocities.
 *
 * Register in your SDF with:
 * @code{.xml}
 * <plugin filename="librobot_plugin.so" name="boris_apartment::RobotPlugin"/>
 * @endcode
 */
class RobotPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
  public:
    void Configure(
        const gz::sim::Entity &p_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &p_ecm,
        gz::sim::EventManager &_eventMgr
    ) override;

    // clang-format off
    void PreUpdate(
        const gz::sim::UpdateInfo &p_info,
        gz::sim::EntityComponentManager &p_ecm
    ) override;
    // clang-format on

  private:
    gz::sim::Model m_model{gz::sim::kNullEntity};
    gz::transport::Node m_node;
    Logger m_logger{"boris_apartment::RobotPlugin"};
    std::mutex m_cmdMutex;
    double m_linearVel{0.0};
    double m_angularVel{0.0};

    void OnCmdVel(const gz::msgs::Twist &p_msg);

    // clang-format off
    void setJointVelocity(
        gz::sim::EntityComponentManager &p_ecm,
        const std::string &p_jointName,
        double p_velocity
    );
    // clang-format on
};

} // namespace boris_apartment
