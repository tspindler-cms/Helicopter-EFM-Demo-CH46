#include <vector>
#include <map>
#include <string>
#include <cassert>
#include <cmath>
#include <iostream>
struct Vec {double x=0,y=0,z=0;};
struct F {Vec pos,dir;};
static std::map<std::string,double> values;
struct EDPARAM {void* getParamHandle(const char* n){return new std::string(n);} void setParamNumber(void* h,double v){values[*(std::string*)h]=v;}};
namespace Convert {double degToRad=3.141592653589793/180;double radToDeg=180/3.141592653589793;double radSecToRPM=60/(2*3.141592653589793),lbft_to_Nm=1.35581795;}
struct {double time=1,mass_kg=100,weight_N=980,rho=1.2,altitudeMSL=100,altitudeAGL=20,totalVelocity_MPS=5;Vec centerOfGravity{1,2,3},airspeed{1,2,3},inertia{10,20,30};} EFMdata;
struct {double CollectiveInput=.5,RollInput=.1,PitchInput=.2,PedalInput=.3,rollOutput=.1,pitchOutput=.2,rollTrim=0,pitchTrim=0;} flightControls;
struct {auto& unit(int){return *this;} double getGovernorTargetN2Pct(){return 100;} bool starterButtonOn=false; int stateForDebug(){return 3;} bool isStarterEngaged(){return false;} double getTorqueRelative(){return .5;} double getAvailablePowerHP(){return 1300;} double throttleInput=1; double getN1RPM(){return .76;} double getEngTorque(){return 100;} double getFuelFlow(){return 50;} double getTOT(){return 400;}} Engine;
struct {double tankFuel(int){return 50;} bool hasEngineFuel(int){return true;} bool valveOpen(int){return true;} bool crossfeedOpen(){return false;} bool tankLow(int){return false;} bool isFuelFlow=true; double getInternalFuel(){return 100;}} Fuel;
struct RotorState {double maxFlapDeg=3,flapLimitBlades=1,inflow=.02,downwash=.03,phaseRad=1.5,minBladeHealth=.8;};
struct {double frontLoadMean=5,aftLoadMean=6,shaftBalanceErrorNm=0; double appliedEnginePowerHP=1200,appliedEngineTorque=10,clutchTorqueMean=8,clutchEngagedFraction=.5,rotorSpeedLimitFraction=0; unsigned rotorSubsteps=1;double rotorStepSeconds=.006;RotorState debugRotorState(bool aft) const {RotorState x;if(aft)x.maxFlapDeg=4;return x;} double pitchAngleRate=0,pitchRateFeedback=0,mixedPitchInput=0; double rollRateFeedback=0,yawRateFeedback=0,mixedRollInput=0,mixedPedalInput=0;std::vector<F> aeroForces;std::vector<Vec> aeroMoments;double getMRomega(){return 27;}double getN2PCT(){return 100;}double getN2omega(){return 27;}double frontThrust(){return 10;}double aftThrust(){return 20;}double frontLoadTorque(){return 5;}double aftLoadTorque(){return 6;}} Aero;
struct {bool idleStopReleased()const{return false;}} throttleControl;
namespace CH46 {constexpr double bladeFirstMassMoment=61.4627876113,tandemBladeInertia=6609.390955;}
// DBG-12: mock SDK reads independently from observed rotor forces.
#include "FlightModel/TandemConfig.h"
enum {ED_FM_ENGINE_0_TORQUE,ED_FM_ENGINE_1_STARTER_RELATED_TORQUE,ED_FM_ENGINE_2_STARTER_RELATED_TORQUE,ED_FM_ENGINE_1_TORQUE,ED_FM_ENGINE_2_TORQUE,ED_FM_ENGINE_1_FUEL_FLOW,ED_FM_ENGINE_2_FUEL_FLOW};
double ed_fm_get_param(unsigned p) {
 if(p==ED_FM_ENGINE_1_TORQUE||p==ED_FM_ENGINE_2_TORQUE)return 1234;
 if(p==ED_FM_ENGINE_1_FUEL_FLOW||p==ED_FM_ENGINE_2_FUEL_FLOW)return .1;
 return 0;
}
struct {int rotorGroupElement(bool aft){return aft?81:63;}} damageModel;
#include "DebugTelemetry.h"
int main(){
 Aero.aeroForces.push_back({{2,4,6},{10,20,40}});
 Aero.aeroMoments.push_back({2,3,4});
 publishDebug(.006);
 assert(Aero.aeroForces.size()==1 && Aero.aeroMoments.size()==1);
 assert(values["CH46_DBG_ROLL_MOMENT"]==22);
 assert(values["CH46_DBG_YAW_MOMENT"]==-7);
 assert(values["CH46_DBG_PITCH_MOMENT"]==4);
 assert(values["CH46_DBG_BODY_UP_FORCE"]==20);
 assert(values["CH46_DBG_VERSION"]==111);
 assert(values["CH46_DBG_APU_TORQUE"]==0);
 assert(values["CH46_DBG_STARTER1_TORQUE"]==0&&values["CH46_DBG_STARTER2_TORQUE"]==0);
 assert(values["CH46_DBG_ENGINE1_TORQUE"]==1234&&values["CH46_DBG_ENGINE2_TORQUE"]==1234);
 assert(values["CH46_DBG_ENGINE_STATE"]==3&&values["CH46_DBG_MODEL_TORQUE_PCT"]==50);
 assert(std::abs(values["CH46_DBG_WORLD_UP_WEIGHT_RATIO"]-20./980.)<1e-12);
 dbgRoll=90;publishDebug(.006);
 assert(std::abs(values["CH46_DBG_WORLD_UP_WEIGHT_RATIO"]+40./980.)<1e-12);
 dbgRoll=0;publishDebug(.006);
 assert(values["CH46_DBG_N1"]==76);
 assert(values["CH46_DBG_POWER_AVAILABLE"]==1300&&values["CH46_DBG_POWER_APPLIED"]==1200);
 assert(values["CH46_DBG_CLUTCH_ENGAGED"]==.5);
 assert(values["CH46_DBG_PITCH_ROOM_NEG"]==1&&values["CH46_DBG_PITCH_ROOM_POS"]==1);
 assert(values["CH46_DBG_SEQUENCE"]==3);
 EFMdata.weight_N=0;publishDebug(.006);
 assert(values["CH46_DBG_UP_WEIGHT_RATIO"]==-1);
 assert(values["CH46_DBG_SEQUENCE"]==4);
 assert(values["CH46_DBG_PARTS_VALID"]==0);
 assert(values["CH46_DBG_FRONT_ROLL_MOMENT"]==0);
 std::cout<<"PASS observer preserves queues, sums moments about CG, N1 units, invalid decomposition\n";
 Aero.aeroForces.clear();Aero.aeroMoments.clear();
 for(int i=0;i<7;++i)Aero.aeroForces.push_back({{double(i+1),double(i+2),double(i+3)},{double(10+i),double(20+i),double(40+i)}});
 for(int i=0;i<3;++i)Aero.aeroMoments.push_back({double(i+2),double(i+3),double(i+4)});
 Aero.rollRateFeedback=-.15;Aero.yawRateFeedback=.20;
 Aero.pitchAngleRate=-.1;Aero.pitchRateFeedback=-.06;Aero.mixedPitchInput=.14;
 publishDebug(.006);
 assert(values["CH46_DBG_PARTS_VALID"]==1);
 assert(std::abs(values["CH46_DBG_PITCH_ANGLE_RATE"]+.1*Convert::radToDeg)<1e-12);
 assert(values["CH46_DBG_PITCH_FEEDBACK"]==-.06&&values["CH46_DBG_PITCH_MIX"]==.14);
 assert(values["CH46_DBG_PITCH_LIMIT"]==.15);
 for(const char* axis : {"ROLL","PITCH","YAW"}){
  std::string name=std::string(axis)+"_MOMENT";
  double parts=values["CH46_DBG_FRONT_"+name]+values["CH46_DBG_AFT_"+name]+values["CH46_DBG_FUSE_"+name];
  assert(std::abs(parts-values["CH46_DBG_"+name])<1e-9);
 }
 assert(Aero.aeroForces.size()==7&&Aero.aeroMoments.size()==3);
 assert(values["CH46_DBG_BODY_FORWARD_FORCE"]==91);
 assert(values["CH46_DBG_BODY_RIGHT_FORCE"]==301);
 assert(std::abs(values["CH46_DBG_BODY_SIDESLIP"]-std::asin(.4)*Convert::radToDeg)<1e-9);
 assert(values["CH46_DBG_FRONT_FLAP_MAX"]==3&&values["CH46_DBG_AFT_FLAP_MAX"]==4);
 assert(values["CH46_DBG_FRONT_BLADE_HEALTH"]==.8);
 assert(values["CH46_DBG_ROLL_LIMIT"]==1&&values["CH46_DBG_YAW_LIMIT"]==1);
 std::cout<<"PASS front/aft/fuselage moments sum to total; snapshots, sideslip and limit flags\n";
 beginDebugRun();
 assert(values["CH46_DBG_VERSION"]==0&&values["CH46_DBG_SEQUENCE"]==0);
 EFMdata.time=3;EFMdata.totalVelocity_MPS=0;
 Aero.rollRateFeedback=-.14;Aero.yawRateFeedback=.19;
 publishDebug(.006);
 assert(values["CH46_DBG_VERSION"]==111&&values["CH46_DBG_SEQUENCE"]==6);
 assert(values["CH46_DBG_RUN_ID"]==1&&values["CH46_DBG_RUN_TIME"]==2);
 assert(values["CH46_DBG_DAMAGE_ELEMENT"]==-1);
 assert(values["CH46_DBG_BODY_SIDESLIP"]==0);
 assert(values["CH46_DBG_ROLL_LIMIT"]==0&&values["CH46_DBG_YAW_LIMIT"]==0);
 std::cout<<"PASS run invalidation/republication, run elapsed time and zero-speed sideslip\n";

 auto expected=values;
 for(auto& f:Aero.aeroForces){f.dir.x*=.5;f.dir.y*=.5;f.dir.z*=.5;}
 for(auto& m:Aero.aeroMoments){m.x*=.5;m.y*=.5;m.z*=.5;}
 auto ff=Aero.aeroForces;auto mm=Aero.aeroMoments;
 Aero.aeroForces.insert(Aero.aeroForces.end(),ff.begin(),ff.end());
 Aero.aeroMoments.insert(Aero.aeroMoments.end(),mm.begin(),mm.end());
 Aero.rotorSubsteps=2;Aero.rotorStepSeconds=.003;publishDebug(.006);
 assert(values["CH46_DBG_PARTS_VALID"]==1);
 for(auto axis:{"ROLL","PITCH","YAW"})for(auto prefix:{"","FRONT_","AFT_","FUSE_"}){
  auto key=std::string("CH46_DBG_")+prefix+axis+"_MOMENT";
  assert(std::abs(values[key]-expected[key])<1e-9);
 }
 assert(Aero.aeroForces.size()==14&&Aero.aeroMoments.size()==6);
 assert(values["CH46_DBG_ROTOR_SUBSTEPS"]==2&&values["CH46_DBG_ROTOR_DT"]==.003);
 std::cout<<"PASS weighted substep telemetry retains total and per-rotor moments without draining queues\n";
 assert(std::abs(values["CH46_DBG_FRONT_LOAD_MEAN"]-5*Convert::lbft_to_Nm)<1e-9);
 assert(std::abs(values["CH46_DBG_AFT_LOAD_MEAN"]-6*Convert::lbft_to_Nm)<1e-9);
 assert(values["CH46_DBG_SHAFT_BALANCE_ERROR"]==0);
 assert(values["CH46_DBG_GOVERNOR_TARGET"]==100&&values["CH46_DBG_GOVERNOR_ERROR"]==0);
 std::cout<<"PASS DBG-13 mean-load units, shaft residual and governor observations\n";

}
