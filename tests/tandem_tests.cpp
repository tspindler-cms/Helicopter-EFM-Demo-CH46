#include "FlightModel/Aero.h"
#include "Systems/Engine.h"
#include <iostream>
#include <stdexcept>
#include <string>
EFM_Globals G_Params;
void require(bool value, const char* name) {
 if(!value) throw std::runtime_error(name);
 std::cout << "PASS " << name << '\n';
}
struct Result { double lift=0, fx=0, side=0, roll=0, yaw=0, pitch=0, front=0, aft=0, torque=0; };
Result run(double collective=.7, double forward=0, double right=0, double leftPedal=0,
 double rho=1.225, double dt=.003, double vx=0, double vy=0, double vz=0, double alt=1000,
 double cgX=0, bool coupled=false) {
 EFMData data; AH6JDamage damage; FlightControls controls;
 data.setMassState(9435,cgX,0,0,24500,185000,172000);
 data.setAtmosphere(alt,288.15,340,rho,101325); damage.init();
 CH46DAero aero(data,damage,controls); TurboshaftEngine engine(data,controls);
 aero.InitializeOn(); engine.initHot();
 controls.CollectiveInput=collective; controls.PitchInput=forward;
 controls.RollInput=right; controls.PedalInput=leftPedal; controls.update();
 aero.setBodyStates(0,0,0,vx,vy,vz,0,0,0,0,0,0,0,0,0,0,0,0);
 Result out; int n=0;
 for(int i=0;i<int(12/dt);++i) {
  data.update(dt);
  if(coupled) { engine.update(dt,true,aero.getN2PCT()); aero.update(engine.getEngTorque(),engine.getAvailablePowerHP()); /* PWR-01 live path */ }
  else aero.evaluateRotors();
  Result r;
  for(auto f:aero.aeroForces) {
   if(!std::isfinite(f.dir.x+f.dir.y+f.dir.z)) throw std::runtime_error("nonfinite force");
   r.fx+=f.dir.x; r.lift+=f.dir.y; r.side+=f.dir.z;
   r.roll+=f.pos.y*f.dir.z-f.pos.z*f.dir.y;
   r.yaw+=f.pos.z*f.dir.x-(f.pos.x-cgX)*f.dir.z;
   r.pitch+=(f.pos.x-cgX)*f.dir.y-f.pos.y*f.dir.x;
  }
  for(auto m:aero.aeroMoments) {r.roll+=m.x;r.yaw+=m.y;r.pitch+=m.z;}
  if(i*dt>10) {
   ++n;out.fx+=r.fx;out.lift+=r.lift;out.side+=r.side;out.roll+=r.roll;out.yaw+=r.yaw;out.pitch+=r.pitch;
   out.front+=aero.frontThrust();out.aft+=aero.aftThrust();out.torque+=aero.frontLoadTorque()+aero.aftLoadTorque();
  }
 }
 out.fx/=n;out.lift/=n;out.side/=n;out.roll/=n;out.yaw/=n;out.pitch/=n;out.front/=n;out.aft/=n;out.torque/=n;
 std::cout<<"sample c="<<collective<<" p="<<forward<<" r="<<right<<" y="<<leftPedal
  <<" rho="<<rho<<" dt="<<dt<<" lift="<<out.lift<<" front="<<out.front<<" aft="<<out.aft
  <<" roll="<<out.roll<<" yaw="<<out.yaw<<" pitch="<<out.pitch<<" side="<<out.side<<" load="<<out.torque<<'\n';
 return out;
}
int main() { try {
 EFMData data;
 data.setAtmosphere(0,288.15,340,1.225,101325);
 require(std::abs(data.rho_SlgFt3-1.225*Convert::kgm3ToSlugFt3)<1e-12,"density units");
 data.setCurrentBodyState(0,0,0,10,20,30,1,2,3,0,0,0,0,0,0,0,0,0);
 require(data.airspeed.x==9 && data.airspeed.y==27 && data.airspeed.z==-18,"wind/body axes");
 require(CH46::transmissibleTorque(-100)==0 && CH46::transmissibleTorque(100)==100,"clutch cannot create positive torque from a negative input");
 auto zero=CH46::mix(.5,0,0,0), pitch=CH46::mix(.5,.5,0,0), yaw=CH46::mix(.5,0,0,.5);
 require(pitch.front.collective<zero.front.collective && pitch.aft.collective>zero.aft.collective,"forward cyclic mixes differential collective");
 require(yaw.front.lateral<0 && yaw.aft.lateral>0,"pedal mixes opposing lateral cyclic");
 require(*std::max_element(_HP_data,_HP_data+_N1_Points)<=2800,"aggregate power ceiling");
 auto base=run(), low=run(.4), thin=run(.7,0,0,0,.8);
 require(base.front>0 && base.aft>0,"both rotors generate upward thrust");
 require(base.lift>low.lift && base.torque>low.torque,"collective increases lift and shaft demand");
 require(thin.lift<base.lift*.8,"reduced density reduces fixed-RPM lift");
 auto fwd=run(.7,.25), back=run(.7,-.25);
 require(fwd.front<base.front && fwd.aft>base.aft && fwd.pitch<base.pitch,"forward cyclic produces nose-down moment");
 require(back.pitch>base.pitch,"aft cyclic produces nose-up moment");
 auto right=run(.7,0,.25), left=run(.7,0,-.25);
 require(right.roll>base.roll && right.side>base.side && left.roll<base.roll,"lateral cyclic rolls and translates in requested direction");
 auto lp=run(.7,0,0,.25), rp=run(.7,0,0,-.25);
 require(lp.yaw>base.yaw && rp.yaw<base.yaw,"pedals yaw in requested direction (DCS +Y is nose-left)");
 require(std::abs(lp.side-base.side)<.1*std::abs(right.side-base.side),"pedals primarily create a yaw couple rather than side thrust");
 auto coarse=run(.7,0,0,0,1.225,.006),fine=run(.7,0,0,0,1.225,.0015);
 require(std::abs(coarse.lift-fine.lift)<.02*base.lift,"fixed-state lift converges across 6, 3, and 1.5 ms steps");
 auto shifted=run(.7,0,0,0,1.225,.003,0,0,0,1000,1);
 require(shifted.pitch<base.pitch-.8*base.lift,"CG shift changes moment arm without shifting model force positions");
 run(.7,0,0,0,1.225,.003,50,0,0);run(.3,0,0,0,1.225,.003,0,-8,0);run(.7,0,0,0,1.225,.003,0,0,10);run(.7,0,0,0,1.225,.003,0,0,0,1);
 require(true,"finite forward-flight, descent, crosswind, and ground-effect forces");
 auto coupled=run(.7,0,0,0,1.225,.003,0,0,0,1000,0,true);
 require(coupled.lift>0 && std::isfinite(coupled.lift),"engine and rotor coupling produces finite lift");
 std::cout<<"All checks passed. These are code checks, not flight-performance validation.\n";
 } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
