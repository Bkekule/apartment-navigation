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

namespace boris_apartment {

/// Joint that drives the robot forward/backward along the x-axis.
static constexpr const char *kDriveJoint = "link_0_JOINT_1";
/// Joint that rotates the robot in-place around the z-axis.
static constexpr const char *kSteerJoint = "link_0_JOINT_0";

/**
 * @brief Gazebo Sim plugin that drives a two-joint robot via `cmd_vel`.
 *
 * Subscribes to `/model/<name>/cmd_vel` (gz.msgs.Twist) and maps:
 * - `linear.x`  → drive joint velocity (m/s)
 * - `angular.z` → steer joint velocity (rad/s)
 *
 * Register in your SDF with:
 * @code{.xml}
 * <plugin filename="librobot_plugin.so" name="boris_apartment::RobotPlugin"/>
 * @endcode
 */
class RobotPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
  private:
    gz::sim::Model model{gz::sim::kNullEntity};
    gz::transport::Node node;

    std::mutex cmdMutex;
    double linearVel{0.0};  ///< Latest linear velocity command (m/s).
    double angularVel{0.0}; ///< Latest angular velocity command (rad/s).

    /**
     * @brief Transport callback — stores the latest Twist command.
     * @param msg Incoming velocity message.
     */
    void OnCmdVel(const gz::msgs::Twist &msg) {
        std::scoped_lock lock(cmdMutex);
        linearVel = msg.linear().x();
        angularVel = msg.angular().z();
    }

  public:
    /**
     * @brief Called once when the plugin is loaded.
     *
     * Resolves the model entity and subscribes to its `cmd_vel` topic.
     *
     * @param _entity  SDF entity that owns this plugin.
     * @param _sdf     Parsed SDF element (unused).
     * @param _ecm     Entity-component manager for the simulation world.
     */
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager & /*_eventMgr*/
    ) override {
        model = gz::sim::Model(_entity);

        // Topic: /model/<robot_name>/cmd_vel
        // Publish a gz.msgs.Twist there to drive the robot.
        std::string topic = "/model/" + model.Name(_ecm) + "/cmd_vel";
        node.Subscribe(topic, &RobotPlugin::OnCmdVel, this);

        gzmsg << "[RobotPlugin] Listening on " << topic << "\n";
    }

    /**
     * @brief Called every simulation step before physics integration.
     *
     * Reads the latest velocity command (under mutex) and forwards it to
     * both joints via `JointVelocityCmd` components.
     *
     * @param _info Simulation step metadata (time, paused flag, …).
     * @param _ecm  Entity-component manager for the simulation world.
     */
    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override {
        if (_info.paused)
            return;

        double lin{0.0};
        double ang{0.0};
        {
            std::scoped_lock lock(cmdMutex);
            lin = linearVel;
            ang = angularVel;
        }

        setJointVelocity(_ecm, kDriveJoint, lin);
        setJointVelocity(_ecm, kSteerJoint, ang);
    }

  private:
    /**
     * @brief Creates or updates a `JointVelocityCmd` component on a named joint.
     *
     * @param _ecm      Entity-component manager.
     * @param jointName Joint name as declared in the model SDF.
     * @param velocity  Target velocity to apply (units depend on joint axis).
     */
    // clang-format off
    void setJointVelocity(
        gz::sim::EntityComponentManager &_ecm,
        const std::string &jointName,
        double velocity
    ) {
        // clang-format on
        auto jointEntity = model.JointByName(_ecm, jointName);
        if (jointEntity == gz::sim::kNullEntity) {
            gzwarn << "[RobotPlugin] Joint not found: " << jointName << "\n";
            return;
        }

        auto *cmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(jointEntity);
        if (cmd == nullptr) {
            _ecm.CreateComponent(jointEntity, gz::sim::components::JointVelocityCmd({velocity}));
        } else {
            cmd->Data()[0] = velocity;
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
