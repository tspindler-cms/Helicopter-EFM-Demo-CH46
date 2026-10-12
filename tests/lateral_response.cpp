#include "FlightModel/Aero.h"
#include <iostream>
#include <cmath>
EFM_Globals G_Params;
int main(int argc,char**argv){double dt=argc>1?atof(argv[1]):.003;
 EFMData d;AH6JDamage damage;FlightControls c;damage.init();d.setMassState(6195,0,0,0,24500,185000,172000);d.setAtmosphere(100,288,340,1.2,101325);CH46DAero a(d,damage,c);a.InitializeOn();
 double roll=0,p=0,r=0,yaw=0,v=0,pdot=0,rdot=0,az=0;double maxRate=0,lastRate=0;
 for(int i=0;i<int(24/dt);i++){
  double t=i*dt;c.CollectiveInput=.54;c.RollInput=(t>=8 && t<9)?.08:0;c.update();d.update(dt);
  // A restricted lateral test: fixed height/forward speed, external gravity.
  // First 8 s settle blades, then integrate lateral speed, roll and yaw only.
  a.setBodyStates(0,0,az,0,0,v,0,0,0,pdot,rdot,0,p,r,0,yaw,0,roll);a.evaluateRotors();a.FuselageModule();double mx=0,my=0,fz=0;
  for(auto f:a.aeroForces){mx+=f.pos.y*f.dir.z-f.pos.z*f.dir.y;my+=f.pos.z*f.dir.x-f.pos.x*f.dir.z;fz+=f.dir.z;}
  for(auto m:a.aeroMoments){mx+=m.x;my+=m.y;}
  if(t>=8){pdot=mx/24500;rdot=my/185000;az=fz/6195;p+=pdot*dt;r+=rdot*dt;roll+=p*dt;yaw+=r*dt;v+=(az+9.80665*sin(roll))*dt;}
  if(t>=9)maxRate=std::max(maxRate,std::abs(p));if(t>=20)lastRate=std::max(lastRate,std::abs(p));
  if(i%int(1/dt)==0)std::cout<<t<<","<<roll*180/M_PI<<","<<p*180/M_PI<<","<<r*180/M_PI<<","<<v<<"\n";
 }
 std::cerr<<"maxrate="<<maxRate*180/M_PI<<" tailrate="<<lastRate*180/M_PI<<"\n";
}
