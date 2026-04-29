#include "kinematics.hh"

namespace boris_apartment::kinematics {

WheelVelocities differentialDrive(double linearVel, double angularVel) {
    return {
        (linearVel - (angularVel * kHalfTrackWidth)) / kWheelRadius,
        (linearVel + (angularVel * kHalfTrackWidth)) / kWheelRadius,
    };
}

} // namespace boris_apartment::kinematics
