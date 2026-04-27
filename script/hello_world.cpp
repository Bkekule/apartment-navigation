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
    /**
     * @brief Called once by Gazebo when the plugin is loaded.
     * @param _entity  SDF entity that owns this plugin (unused).
     * @param _sdf     Parsed SDF element (unused).
     * @param _ecm     Entity-component manager (unused).
     * @param _eventMgr Event manager (unused).
     */
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr
    ) override {
        gzmsg << "======= Welcome to Boris Apartment's World! =======\n";
    }

    /**
     * @brief Called every simulation step before physics integration. Called repeadedly, even when simulation is
     * paused.
     *
     * Logs the current simulation time at most once per simulated second.
     *
     * @param p_info Simulation step metadata (time, paused flag, …).
     * @param _ecm   Entity-component manager (unused).
     */
    // clang-format off
    void PreUpdate(
        const gz::sim::UpdateInfo &p_info,
        gz::sim::EntityComponentManager &_ecm
    ) override {
        // clang-format on
        // Log once per second to avoid flooding the console.
        auto l_simSeconds = std::chrono::duration<double>(p_info.simTime).count();
        if (static_cast<int>(l_simSeconds) > m_lastLoggedSecond) {
            m_lastLoggedSecond = static_cast<int>(l_simSeconds);
            gzmsg << "PreUpdate running at sim time: " << l_simSeconds << "s\n";
        }
    }

  private:
    int m_lastLoggedSecond{-1}; ///< Last simulation second that was logged, to throttle output.
};
} // namespace boris_apartment

// Register the plugin with gz-sim.
// Arguments: class, interfaces it implements.
// clang-format off
GZ_ADD_PLUGIN(
    boris_apartment::WorldPluginMyRobot,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)
// clang-format on
