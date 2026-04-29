add_subdirectory(kinematics)

add_library(robot_plugin MODULE
    plugin/robot_plugin.cpp
    plugin/robot_plugin_register.cpp)

# Expose script/robot/ so includes like "plugin/robot_plugin.hh" and
# "kinematics/kinematics.hh" resolve correctly within this target.
target_include_directories(robot_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(robot_plugin
    PRIVATE robot_kinematics
    PRIVATE boris_common
    PRIVATE gz-sim8::gz-sim8
    PRIVATE gz-plugin2::register)
