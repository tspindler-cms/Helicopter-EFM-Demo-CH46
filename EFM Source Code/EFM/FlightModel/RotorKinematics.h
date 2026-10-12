#pragma once
#include "../include/ED_FM_Utility.h"

namespace CH46 {
// KIN-01 (0.1.5): DCS supplies inertial acceleration resolved in body axes,
// not the time derivative of the components of body-axis velocity. Therefore
// do NOT add omega x translational airspeed again. The recorded 0.1.4 flight
// independently agrees with a_body = force/mass + gravity_body.
//
// A point fixed to the airframe has acceleration:
//   a_hub = a_cg + alpha x offset + omega x (omega x offset).
// The blade equations need specific acceleration, so subtract gravity once.
// All vectors must use the SAME right-handed frame and length units. The
// caller uses forward/right/down axes and feet, including the aft reflection.
// Air-relative velocity belongs to the aerodynamic inflow calculation, not
// this rigid-body acceleration calculation. No thrust or control gain is added.
inline Vec3 hubSpecificAcceleration(const Vec3& acceleration,
                                   const Vec3& gravity,
                                   const Vec3& omega,
                                   const Vec3& alpha,
                                   const Vec3& offset) {
    const Vec3 tangential = cross(alpha, offset);
    const Vec3 centripetal = cross(omega, cross(omega, offset));
    return Vec3(acceleration.x - gravity.x + tangential.x + centripetal.x,
                acceleration.y - gravity.y + tangential.y + centripetal.y,
                acceleration.z - gravity.z + tangential.z + centripetal.z);
}
}
