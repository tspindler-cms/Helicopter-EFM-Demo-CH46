#pragma once
#include <algorithm>
#include <cmath>

// Experimental CH-46D conversion. See TANDEM_PROTOTYPE.md for provenance
// and limitations. Geometry comes from the supplied VWV Lua, not measured EDM.
namespace CH46 {
constexpr double pi = 3.14159265358979323846;
constexpr double nominalRotorOmega = 264.0 * 2.0 * pi / 60.0;
constexpr double frontHubX = 5.5208;
constexpr double frontHubY = 3.0342;
constexpr double aftHubX = -4.7625;
constexpr double aftHubY = 4.438;
constexpr double frontShaftTilt = -8.0 * pi / 180.0;
constexpr double aftShaftTilt = -5.5 * pi / 180.0;
// Navy museum identifies 2 x 1,400 shp T58-GE-10. This is an aggregate
// power ceiling, NOT a validated engine map or transmission/time limit.
constexpr double aggregatePowerHP = 2800.0;
// API-02 (0.1.9): engine OUTPUT-shaft reference, not rotor speed.
// T58-GE-10: 19,500 rpm at both 1,400 shp military and 1,250 shp normal
// ratings in USN SH-3H SAC, 3 April 1972 (same engine, different airframe).
// This is reporting/gearing conversion only; no engine governor change.
constexpr double engineOutputReferenceRPM = 19500.0;
// API-02: 26,300 rpm gas-generator reference from the T58/CT58 family
// certification data. Display reference remains provisional for the exact
// Vietnam CH-46D installation; percent-N1 physics is unchanged.
constexpr double engineCoreReferenceRPM = 26300.0;
// FUEL-04 (0.1.9): match the supplied VWV Database/vwv_ch46d_base.lua
// M_fuel_max=1080 kg. This removes the inherited 401+412 lb capacity cap;
// it is module consistency, NOT independently verified Vietnam tank data.
// Two equal bookkeeping reservoirs feed their own engines (FUEL-05). Their
// physical locations, transfer plumbing and weight distribution are not modeled.
constexpr double internalFuelCapacityKg = 1080.0;
constexpr double fuelReservoirCapacityKg = internalFuelCapacityKg / 2.0;

// GEO-02: project a body-frame offset into world vertical. AGL is measured
// at the aircraft reference point; hub height changes with pitch and bank.
// z is right, y is up. Heading does not affect vertical height.
inline double verticalOffset(double x, double y, double z, double pitch, double roll) {
    return x*std::sin(pitch) + std::cos(pitch)*(y*std::cos(roll)-z*std::sin(roll));
}

inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}
// FCS-03 (0.1.12A): retain full control travel while softening the centre of
// the cyclic and pedal axes for hover work.  This is an EFM response curve,
// not a replacement for the user's DCS axis settings.
inline double precisionAxis(double input) {
    const double x=clamp(input,-1.0,1.0);
    return x*(0.65+0.35*x*x);
}
struct RotorControls { double collective, lateral, longitudinal; };
struct Controls { RotorControls front, aft; };
inline Controls mix(double collective, double forward, double right, double leftPedal) {
    // Gains are provisional blade-pitch degrees, not NATOPS rigging values.
    const double common = 2.0 + 19.0 * clamp(collective, 0.0, 1.0);
    const double dcp = 4.0 * clamp(forward, -1.0, 1.0);
    const double lateral = 8.0 * clamp(right, -1.0, 1.0);
    const double yaw = 4.0 * clamp(leftPedal, -1.0, 1.0);
    return {{clamp(common - dcp, -2.0, 25.0), lateral - yaw, 0.0},
            {clamp(common + dcp, -2.0, 25.0), lateral + yaw, 0.0}};
}
// 0.1.2 TEST ONLY: bounded cyclic rate feedback, not calibrated CH-46D AFCS.
// Targets the observed growing lateral oscillation with near-neutral pedals.
// Gains: normalized input per rad/s. Limits: fractions of full pilot input.
// No attitude/heading target, integrator, extra force or engine-power change.
// FCS-02: kinematic pitch-angle rate, including yaw while banked.
// This is a provisional helper, not a claim of period SAS control-law fidelity.
inline double pitchAngleRate(double bodyPitchRadSec, double yawDcsRadSec, double rollRad) {
    return bodyPitchRadSec*std::cos(rollRad)+yawDcsRadSec*std::sin(rollRad);
}
struct RateFeedback { double roll, pedal, pitch; };
inline RateFeedback rateFeedback(double rollRadSec, double yawDcsRadSec,
                                  double rotorFraction, double pitchAngleRateRadSec=0.0) {
#ifdef CH46_RATE_DAMPING_OFF
    return {0.0, 0.0, 0.0};
#else
    // Fade with rotor speed; no control correction with rotors stopped.
    const double authority=clamp((rotorFraction-0.2)/0.5,0.0,1.0);
    // FCS-03: stronger bounded damping addresses the recorded progressive
    // bank/dive without adding attitude hold or changing full pilot authority.
    return {authority*clamp(-0.9*rollRadSec,-0.18,0.18),
            authority*clamp(-1.5*yawDcsRadSec,-0.22,0.22),
            // FCS-02 (0.1.6 TEST): bounded pitch-angle-rate feedback, gain in
            // normalized input per rad/s. Positive horizon-relative pitch-angle rate is nose-up;
            // positive mixer pitch input is nose-down. No attitude target,
            // integrator, auto-level or extra force. Not calibrated CH-46D SAS.
            authority*clamp(0.8*pitchAngleRateRadSec,-0.18,0.18)};
#endif
}
inline double transmissibleTorque(double torque) {
    return std::max(0.0, torque); // an overrunning clutch cannot drive backwards
}
}
