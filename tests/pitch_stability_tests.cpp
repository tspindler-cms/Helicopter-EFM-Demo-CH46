// FCS-02: verify correction direction, active EFM mixing, authority bounds
// and reset. Quaternion kinematics are tested in the separate free-response
// harness. No real-aircraft gain claim.
#include "FlightModel/Aero.h"
#include <cassert>
#include <iostream>
EFM_Globals G_Params;
int main(){
 EFMData d;AH6JDamage damage;FlightControls c;damage.init();
 d.setMassState(6195,0,0,0,24500,185000,172000);
 d.setAtmosphere(1000,288.15,340,1.19788,101325);d.update(.003);
 CH46DAero a(d,damage,c);a.InitializeOn();
 auto sample=[&](double q,double yaw,double roll,double pilot){
  c.PitchInput=pilot;c.CollectiveInput=.5;c.update();
  a.setBodyStates(0,0,0,30,0,0,0,0,0,0,0,0,0,yaw,q,0,0,roll);
  a.evaluateRotors();assert(c.PitchInput==pilot&&c.pitchOutput==pilot);
  assert(std::isfinite(a.pitchAngleRate+a.pitchRateFeedback+a.mixedPitchInput));
  assert(std::abs(a.pitchRateFeedback)<=.18&&std::abs(a.mixedPitchInput)<=1);
 };
 sample(.1,0,0,-.05);
#ifdef CH46_RATE_DAMPING_OFF
 assert(a.pitchRateFeedback==0&&a.mixedPitchInput==-.05);
#else
 assert(a.pitchRateFeedback>0&&a.mixedPitchInput>CH46::precisionAxis(-.05)); // nose-up rate -> nose-down correction
 sample(.1,-.2,CH46::pi/4,-.05);
 assert(a.pitchAngleRate<0&&a.pitchRateFeedback<0&&a.mixedPitchInput<-.05);
 sample(-.1,0,0,0);assert(a.pitchRateFeedback<0); // falling nose -> aft correction
 sample(20,0,0,.95);assert(a.pitchRateFeedback==.18&&a.mixedPitchInput==1);
 sample(-20,0,0,-.95);assert(a.pitchRateFeedback==-.18&&a.mixedPitchInput==-1);
 sample(1,-1,CH46::pi/2,0);assert(a.pitchRateFeedback<0); // no division by cos(bank)
#endif
 a.InitializeOff();sample(.1,-.2,CH46::pi/4,-.05);
 assert(a.pitchRateFeedback==0&&a.mixedPitchInput==CH46::precisionAxis(-.05));
 a.InitializeOn();assert(a.pitchAngleRate==0&&a.pitchRateFeedback==0&&a.mixedPitchInput==0);
 std::cout<<"PASS pitch helper direction, bank/yaw coupling, limits, raw controls, stopped rotor and reset\n";
}
