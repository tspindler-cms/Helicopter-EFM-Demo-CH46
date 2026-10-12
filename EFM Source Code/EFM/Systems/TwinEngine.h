#pragma once
#include "Engine.h"

// ENG-01: independent gas-generator, ignition and thermal states. Both still
// drive the existing common turbine/combining-shaft and single-clutch model.
// This does NOT model separate free-turbine inertias, accessory drives or
// validated OEI performance. A common throttle remains compatible with the
// supplied cockpit. Independent fuel valves and starter commands are provided.
class TwinEngine {
    TurboshaftEngine first_, second_;
    bool starter_[2]{false,false};
public:
    double throttleInput=0;
    bool starterButtonOn=false;
    TwinEngine(EFMData& data,FlightControls& controls)
        :first_(data,controls,.5,true),second_(data,controls,.5,false){}
    TurboshaftEngine& unit(int i){return i==0?first_:second_;}
    const TurboshaftEngine& unit(int i)const{return i==0?first_:second_;}
    void initCold(){first_.initCold();second_.initCold();reset(false);}
    void initHot(){first_.initHot();second_.initHot();reset(true);}
    void reset(bool hot){throttleInput=hot?1:0;starterButtonOn=false;starter_[0]=starter_[1]=false;}
    void setStarter(int i,bool on){if(i>=0&&i<2)starter_[i]=on;}
    void update(double dt,bool fuel1,bool fuel2,double n2){
        first_.throttleInput=second_.throttleInput=throttleInput;
        first_.starterButtonOn=starterButtonOn||starter_[0];
        second_.starterButtonOn=starterButtonOn||starter_[1];
        first_.update(dt,fuel1,n2);second_.update(dt,fuel2,n2);
        G_Params.cautionLight[CL_EngOut]=first_.stateForDebug()==0||second_.stateForDebug()==0;
    }
    void update(double dt,bool fuel,double n2){update(dt,fuel,fuel,n2);}
    double getN1RPM(){return .5*(first_.getN1RPM()+second_.getN1RPM());}
    double getGeneratorDriveN1(){return std::max(first_.getN1RPM(),second_.getN1RPM());}
    double getTOT(){return std::max(first_.getTOT(),second_.getTOT());}
    double getOilPress_Pa(){return .5*(first_.getOilPress_Pa()+second_.getOilPress_Pa());}
    double getFuelFlow(){return first_.getFuelFlow()+second_.getFuelFlow();}
    double getEngTorque(){return first_.getEngTorque()+second_.getEngTorque();}
    double getTorque_Nm(){return first_.getTorque_Nm()+second_.getTorque_Nm();}
    double getAvailablePowerHP()const{return first_.getAvailablePowerHP()+second_.getAvailablePowerHP();}
    double getTorqueRelative(){return .5*(first_.getTorqueRelative()+second_.getTorqueRelative());}
    bool isStarterEngaged()const{return first_.isStarterEngaged()||second_.isStarterEngaged();}
    int stateForDebug()const{return std::max(first_.stateForDebug(),second_.stateForDebug());}
    double getGovernorTargetN2Pct()const{
        const double a=first_.getGovernorTargetN2Pct(),b=second_.getGovernorTargetN2Pct();
        const int running=(a>0)+(b>0);return running?(a+b)/running:0;
    }
};
