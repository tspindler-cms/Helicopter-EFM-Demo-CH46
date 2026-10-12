#pragma once
#include <algorithm>
#include <cmath>

namespace CH46 {
// MASS-01 (0.1.8 TEST): a NON-CALIBRATED, nonnegative blade mass model.
// Retain the prototype's 155 lbf weight and 1044 slug-ft^2 hinge inertia.
// Represent them by uniform mass from hinge to tip plus a point tip mass.
// Solve the tip fraction from I = m*L^2*((1-f)/3 + f), then derive M1.
// The previous M1=120.4 violated M1^2 <= m*I and put the CG past the tip.
// This assumption removes that contradiction; it is not measured CH-46D data.
constexpr double bladeRadiusFt = 25.0;
constexpr double bladeHingeOffsetFt = .46;
constexpr double bladeSpanFt = bladeRadiusFt - bladeHingeOffsetFt;
constexpr double bladeWeightLbf = 155.0;
constexpr double bladeMassSlug = bladeWeightLbf / 32.174;
constexpr double bladeHingeInertia = 1044.0;
constexpr double bladeTipMassFraction =
    1.5*(bladeHingeInertia/(bladeMassSlug*bladeSpanFt*bladeSpanFt)-1.0/3.0);
constexpr double bladeFirstMassMoment =
    bladeMassSlug*bladeSpanFt*(.5+.5*bladeTipMassFraction);
// Parallel-axis theorem for six blades at zero flap. Hub/transmission inertia
// is not fabricated; this is a blade-only estimate plus the separate JE model.
constexpr double tandemBladeInertia = 6.0*(bladeHingeInertia
    +2.0*bladeHingeOffsetFt*bladeFirstMassMoment
    +bladeHingeOffsetFt*bladeHingeOffsetFt*bladeMassSlug);
static_assert(bladeTipMassFraction >= 0 && bladeTipMassFraction <= 1,
              "blade mass assumption requires nonnegative component masses");
static_assert(bladeFirstMassMoment*bladeFirstMassMoment <= bladeMassSlug*bladeHingeInertia,
              "blade first/second moments violate Cauchy-Schwarz");

// PWR-01: P=Q*omega, with HP and ft-lb. At/below nominal RPM preserve the
// previous finite torque ceiling instead of dividing by a near-zero speed.
// Above nominal RPM reduce torque so instantaneous applied power cannot exceed
// its budget. The low-speed torque cap is provisional, not a T58 engine map.
inline double powerLimitedTorque(double powerHP, double omega, double nominalOmega) {
    if(!std::isfinite(powerHP)||!std::isfinite(omega)||!std::isfinite(nominalOmega)
       ||powerHP<=0||nominalOmega<=0) return 0;
    return powerHP*550.0/std::max(nominalOmega,std::max(0.0,omega));
}

struct ShaftStep {
    double rotorOmega, engineOmega, clutchTorque, engagementLoss;
    bool engaged;
};
// CLUTCH-01: advance the shafts freely, then impose the one-way constraint
// omega_engine <= omega_rotor. A faster engine transfers a NONNEGATIVE impulse
// to the rotor. Common speed conserves angular momentum; engagement dissipates
// energy. A rotor that accelerates faster can overrun even with positive engine
// torque. Previously engine speed alone plus positive torque forced locking.
// Torque: ft-lb; inertia: slug-ft^2; angular speed: rad/s; loss: ft-lb.
inline ShaftStep advanceShafts(double rotorOmega,double engineOmega,
        double rotorTorque,double engineTorque,double rotorInertia,
        double engineInertia,double dt) {
    const double r=std::max(0.0,rotorOmega+rotorTorque/rotorInertia*dt);
    const double e=std::max(0.0,engineOmega+engineTorque/engineInertia*dt);
    if(e<=r) return {r,e,0,0,false};
    const double reduced=rotorInertia*engineInertia/(rotorInertia+engineInertia);
    const double impulse=reduced*(e-r);
    const double common=(rotorInertia*r+engineInertia*e)/(rotorInertia+engineInertia);
    return {common,common,impulse/dt,.5*reduced*(e-r)*(e-r),true};
}
}
