#include "logging.hh"

#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <mutex>
#include <string>

static const char *kLeftWheelJoint = "deck_to_left_wheel";
static const char *kRightWheelJoint = "deck_to_right_wheel";
/// Wheel radius in metres.
static const double kWheelRadius = 0.2;
/// Half the track width (distance from body center to wheel axle) in metres.
static const double kHalfTrackWidth = 0.550019;

namespace boris_apartment {
/**
 * @brief Gazebo Sim plugin for differential drive robot via `cmd_vel`.
 *
 * Subscribes to `/model/<name>/cmd_vel` (gz.msgs.Twist) and uses differential
 * drive kinematics to map to left and right wheel angular velocities:
 * - `linear.x`  → forward/backward speed (m/s)
 * - `angular.z` → rotation speed (rad/s)
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
    /**
     * @brief Called once when the plugin is loaded.
     *
     * Resolves the model entity and subscribes to its `cmd_vel` topic.
     *
     * @param p_entity   SDF entity that owns this plugin.
     * @param _sdf       Parsed SDF element (unused).
     * @param p_ecm      Entity-component manager for the simulation world.
     * @param _eventMgr  Event manager (unused).
     */
    void Configure(
        const gz::sim::Entity &p_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &p_ecm,
        gz::sim::EventManager &_eventMgr
    ) override {
        m_model = gz::sim::Model(p_entity);

        // Topic: /model/<robot_name>/cmd_vel
        // Publish a gz.msgs.Twist there to drive the robot.
        std::string l_topic = "/model/" + m_model.Name(p_ecm) + "/cmd_vel";
        m_node.Subscribe(l_topic, &RobotPlugin::OnCmdVel, this);

        m_logger.info() << "Listening on " << l_topic << "\n";
    }

    /**
     * @brief Called every simulation step before physics integration. Called repeadedly, even when simulation is
     * paused.
     *
     * Reads the latest velocity command (under mutex) and applies differential
     * drive kinematics to compute left and right wheel velocities, then commands
     * both wheels via `JointVelocityCmd` components.
     *
     * @param p_info Simulation step metadata (time, paused flag, …).
     * @param p_ecm  Entity-component manager for the simulation world.
     */
    // clang-format off
    void PreUpdate(
        const gz::sim::UpdateInfo &p_info,
        gz::sim::EntityComponentManager &p_ecm
    ) override {
        // clang-format on
        if (p_info.paused)
            return;

        double l_lin{0.0};
        double l_ang{0.0};
        {
            std::scoped_lock l_lock(m_cmdMutex);
            l_lin = m_linearVel;
            l_ang = m_angularVel;
        }

        // Differential drive kinematics: map Twist to wheel angular velocities
        double l_vLeft = (l_lin - (l_ang * kHalfTrackWidth)) / kWheelRadius;
        double l_vRight = (l_lin + (l_ang * kHalfTrackWidth)) / kWheelRadius;
        setJointVelocity(p_ecm, kLeftWheelJoint, l_vLeft);
        setJointVelocity(p_ecm, kRightWheelJoint, l_vRight);
    }

  private:
    gz::sim::Model m_model{gz::sim::kNullEntity};
    gz::transport::Node m_node;
    Logger m_logger{"boris_apartment::RobotPlugin"};
    std::mutex m_cmdMutex;
    double m_linearVel{0.0};  ///< Latest linear velocity command (m/s).
    double m_angularVel{0.0}; ///< Latest angular velocity command (rad/s).

    /**
     * @brief Transport callback — stores the latest Twist command.
     * @param p_msg Incoming velocity message.
     */
    void OnCmdVel(const gz::msgs::Twist &p_msg) {
        std::scoped_lock l_lock(m_cmdMutex);
        m_linearVel = p_msg.linear().x();
        m_angularVel = p_msg.angular().z();
    }

    /**
     * @brief Creates or updates a `JointVelocityCmd` component on a named joint.
     *
     * @param p_ecm         Entity-component manager.
     * @param p_jointName   Joint name as declared in the model SDF.
     * @param p_velocity    Target velocity to apply (units depend on joint axis).
     */
    // clang-format off
    void setJointVelocity(
        gz::sim::EntityComponentManager &p_ecm,
        const std::string &p_jointName,
        double p_velocity
    ) {
        // clang-format on
        auto l_jointEntity = m_model.JointByName(p_ecm, p_jointName);
        if (l_jointEntity == gz::sim::kNullEntity) {
            m_logger.warn() << "Joint not found: " << p_jointName << "\n";
            return;
        }

        auto *l_cmd = p_ecm.Component<gz::sim::components::JointVelocityCmd>(l_jointEntity);
        if (l_cmd == nullptr) {
            p_ecm.CreateComponent(l_jointEntity, gz::sim::components::JointVelocityCmd({p_velocity}));
        } else {
            l_cmd->Data()[0] = p_velocity;
        }
    }
};
} // namespace boris_apartment

// clang-format off
GZ_ADD_PLUGIN(
    boris_apartment::RobotPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)
// clang-format on
