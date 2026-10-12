#define __declspec(x)
#include "Systems/Engine.h"
#include <cassert>
#include <cmath>
#include <iostream>

EFM_Globals G_Params;

int main()
{
    EFMData data;
    data.ambientTemp_C = 15.0;
    data.altitudeMSL_FT = 100.0;
    FlightControls controls;
    controls.CollectiveInput = 0.52;
    TurboshaftEngine engine(data, controls);
    engine.initHot();

    // Approximate the measured 0.1.12A hover equilibrium: about 76% N1 held
    // 100% N2 under the recorded rotor load. This checks the governor loop,
    // not rotor aerodynamics.
    double n2 = 104.3;
    constexpr double dt = 0.006;
    for (int i = 0; i < 10000; ++i)
    {
        engine.update(dt, true, n2);
        const double equilibriumN2 = 100.0 + 1.25 * (100.0 * engine.getN1RPM() - 76.0);
        n2 += (equilibriumN2 - n2) * dt / 1.8;
        assert(std::isfinite(n2));
    }
    assert(std::abs(n2 - engine.getGovernorTargetN2Pct()) < 0.25);

    controls.CollectiveInput = 0.75;
    double minimumN2 = n2;
    double maximumN2 = n2;
    for (int i = 0; i < 2500; ++i)
    {
        engine.update(dt, true, n2);
        // Higher collective increases required N1 by approximately the same
        // feed-forward slope used by the existing model.
        const double requiredN1 = 76.0 + 27.0 * (controls.CollectiveInput - 0.52);
        const double equilibriumN2 = 100.0 + 1.25 * (100.0 * engine.getN1RPM() - requiredN1);
        n2 += (equilibriumN2 - n2) * dt / 1.8;
        minimumN2 = std::min(minimumN2, n2);
        maximumN2 = std::max(maximumN2, n2);
    }
    assert(minimumN2 > 96.0);
    assert(maximumN2 < 103.0);
    assert(std::abs(n2 - engine.getGovernorTargetN2Pct()) < 0.5);

    std::cout << "PASS governor removes steady hover overspeed and remains bounded after a collective step\n";
}
