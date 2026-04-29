add_library(robot_kinematics OBJECT kinematics.cpp)

# Expose script/robot/ as the include root so consumers use #include "kinematics/kinematics.hh"
target_include_directories(robot_kinematics PUBLIC ${CMAKE_SOURCE_DIR}/script/robot)
