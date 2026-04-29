#include "plugin/robot_plugin.hh"

#include "kinematics/kinematics.hh"

#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <mutex>

static const char *kLeftWheelJoint = "deck_to_left_wheel";
static const char *kRightWheelJoint = "deck_to_right_wheel";

namespace boris_apartment {

void RobotPlugin::Configure(
    const gz::sim::Entity &p_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &p_ecm,
    gz::sim::EventManager &_eventMgr
) {
    m_model = gz::sim::Model(p_entity);

    std::string l_topic = "/model/" + m_model.Name(p_ecm) + "/cmd_vel";
    m_node.Subscribe(l_topic, &RobotPlugin::OnCmdVel, this);

    m_logger.info() << "Listening on " << l_topic << "\n";
}

// clang-format off
void RobotPlugin::PreUpdate(
    const gz::sim::UpdateInfo &p_info,
    gz::sim::EntityComponentManager &p_ecm
) {
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

    auto [l_vLeft, l_vRight] = kinematics::differentialDrive(l_lin, l_ang);
    setJointVelocity(p_ecm, kLeftWheelJoint, l_vLeft);
    setJointVelocity(p_ecm, kRightWheelJoint, l_vRight);
}

void RobotPlugin::OnCmdVel(const gz::msgs::Twist &p_msg) {
    std::scoped_lock l_lock(m_cmdMutex);
    m_linearVel = p_msg.linear().x();
    m_angularVel = p_msg.angular().z();
}

// clang-format off
void RobotPlugin::setJointVelocity(
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

} // namespace boris_apartment
