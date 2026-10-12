#pragma once
#include <cmath>
namespace CH46 {
struct SlipBall { double degrees; bool valid; };
// GAUGE-01: ball follows apparent gravity, not aerodynamic sideslip angle.
// DCS body axes: x forward, y up, z right. Subtract world gravity projected
// into the body, then show the ball opposite lateral specific acceleration.
// Zero/negative normal load cannot provide the usual upright ball indication.
inline SlipBall slipBall(double ay,double az,double pitch,double roll) {
    const double up=ay+9.80665*std::cos(pitch)*std::cos(roll);
    const double right=az-9.80665*std::cos(pitch)*std::sin(roll);
    if(!std::isfinite(up)||!std::isfinite(right)||up<.5)return {0,false};
    return {-std::atan2(right,up)*180.0/3.14159265358979323846,true};
}
}
