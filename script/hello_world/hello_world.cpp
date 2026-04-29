#include "logging.hh"

#include <chrono>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Element.hh>

namespace boris_apartment {

/**
 * @brief Gazebo Sim world plugin that logs simulation time once per second.
 *
 * A minimal example plugin. Register in your SDF with:
 * @code{.xml}
 * <plugin filename="libhello_world.so" name="boris_apartment::WorldPluginMyRobot"/>
 * @endcode
 */
class WorldPluginMyRobot : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate {
  public:
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr
    ) override {
        m_logger.info() << "======= Welcome to Boris Apartment's World! =======\n";
    }

    // clang-format off
    void PreUpdate(
        const gz::sim::UpdateInfo &p_info,
        gz::sim::EntityComponentManager &_ecm
    ) override {
        // clang-format on
        // Log once per 1000 seconds to avoid flooding the console.
        auto l_simSeconds = std::chrono::duration<double>(p_info.simTime).count();
        if (static_cast<int>(l_simSeconds / 1000.0) > m_lastLoggedSecond) {
            m_lastLoggedSecond = static_cast<int>(l_simSeconds / 1000.0);
            m_logger.info() << "PreUpdate running at sim time: " << l_simSeconds << "s\n";
        }
    }

  private:
    Logger m_logger{"boris_apartment::WorldPluginMyRobot"};
    int m_lastLoggedSecond{-1};
};

} // namespace boris_apartment

// clang-format off
GZ_ADD_PLUGIN(
    boris_apartment::WorldPluginMyRobot,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)
// clang-format on
