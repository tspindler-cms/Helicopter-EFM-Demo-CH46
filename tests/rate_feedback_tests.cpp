#include "FlightModel/TandemConfig.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
 assert(CH46::precisionAxis(0)==0);
 assert(std::abs(CH46::precisionAxis(.1)-.06535)<1e-12);
 assert(CH46::precisionAxis(1)==1&&CH46::precisionAxis(-1)==-1);
 for(double p:{-1.,-.1,0.,.1,1.})for(double r:{-1.,-.1,0.,.1,1.}){
  auto f=CH46::rateFeedback(p,r,1.0);
  assert(std::abs(f.roll)<=.18 && std::abs(f.pedal)<=.22);
  assert(f.roll*p<=0 && f.pedal*r<=0);
  assert(CH46::rateFeedback(p,r,0).roll==0 && CH46::rateFeedback(p,r,0).pedal==0);
 }
 auto a=CH46::rateFeedback(.1,.1,1),b=CH46::rateFeedback(.1,.1,.45);
#ifdef CH46_RATE_DAMPING_OFF
 assert(a.roll==0 && a.pedal==0);
#else
 assert(std::abs(a.roll+.09)<1e-12 && std::abs(a.pedal+.15)<1e-12);
 assert(std::abs(b.roll-a.roll*.5)<1e-12 && std::abs(b.pedal-a.pedal*.5)<1e-12);
 // Rotor mixer direction: right roll produces reduced common lateral cyclic;
 // positive DCS yaw produces a negative pedal correction, preserving signs.
 auto m=CH46::mix(.54,0,a.roll,a.pedal);
 assert((m.front.lateral+m.aft.lateral)/2<0);
 assert(m.aft.lateral-m.front.lateral<0);
#endif
 std::cout<<"PASS precision axis, bounded rate feedback, sign, rotor-speed fade and mixer direction\n";
}
