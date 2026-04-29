#pragma once

namespace boris_apartment::kinematics {

constexpr double kWheelRadius = 0.2;
constexpr double kHalfTrackWidth = 0.550019;

struct WheelVelocities {
    double left;
    double right;
};

WheelVelocities differentialDrive(double linearVel, double angularVel);

} // namespace boris_apartment::kinematics
