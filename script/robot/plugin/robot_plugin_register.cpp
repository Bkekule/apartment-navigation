#include "plugin/robot_plugin.hh"

#include <gz/plugin/Register.hh>

// clang-format off
GZ_ADD_PLUGIN(
    boris_apartment::RobotPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate
)
// clang-format on
