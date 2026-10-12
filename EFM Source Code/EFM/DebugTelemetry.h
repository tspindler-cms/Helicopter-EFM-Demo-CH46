// BUILD-03 / DBG-09 (0.1.11 TEST): identity 111; DBG-11 power/clutch/mixer diagnostics appended, 153 total.
#pragma once
#include "FlightModel/GaugeMath.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <cmath>
// DEBUG 0.1.11 TEST: Observes state only. Cache handles once; never consume DCS force queues.
// Capture immediately after simulation, before DCS pops forces/moments.
static double dbgRoll=0, dbgPitch=0, dbgP=0, dbgQ=0, dbgR=0, dbgClimb=0;
static char dbgText[2048] = "CH46 DEBUG 0.1.11 TEST: waiting for simulation";
// DBG-01/02/03: observations only. TIME retains its previous meaning.
static double dbgAx=0,dbgAy=0,dbgAz=0,dbgPdot=0,dbgQdot=0,dbgRdot=0;
static double dbgRunId=0,dbgRunStart=0;
static double dbgDamageCount=0,dbgDamageElement=-1,dbgDamageFactor=1,dbgDamageTime=-1;
static void beginDebugRun() {
    ++dbgRunId; dbgRunStart=EFMdata.time;
    dbgDamageCount=0;dbgDamageElement=-1;dbgDamageFactor=1;dbgDamageTime=-1;
    dbgAx=dbgAy=dbgAz=dbgPdot=dbgQdot=dbgRdot=0;
    // Invalidate held telemetry until the next simulation step publishes it.
    EDPARAM api;
    api.setParamNumber(api.getParamHandle("CH46_DBG_VERSION"),0);
    api.setParamNumber(api.getParamHandle("CH46_DBG_SEQUENCE"),0);
}
static void publishDebug(double dt) {
    static EDPARAM api;
    static void* version=api.getParamHandle("CH46_DBG_VERSION");
    static void* sequence=api.getParamHandle("CH46_DBG_SEQUENCE");
    static unsigned long seq=0;
    // Sum exported aerodynamic reactions including r cross F; gravity is DCS-owned.
    double fy=0, mx=0, my=0, mz=0;
    for (const auto& f : Aero.aeroForces) {
        const double x=f.pos.x-EFMdata.centerOfGravity.x;
        const double y=f.pos.y-EFMdata.centerOfGravity.y;
        const double z=f.pos.z-EFMdata.centerOfGravity.z;
        fy+=f.dir.y;
        mx+=y*f.dir.z-z*f.dir.y;
        my+=z*f.dir.x-x*f.dir.z;
        mz+=x*f.dir.y-y*f.dir.x;
    }
    for(const auto& m : Aero.aeroMoments) {mx+=m.x; my+=m.y; mz+=m.z;}
    // DBG-10 / NUM-01: each internal step exports 3 front blade forces, 3 aft, then fuselage;
    // corresponding moments are front, aft, fuselage. Components are weighted
    // by substep duration. Sum every group without consuming the queues. Mark invalid if that
    // contract changes or the observer is called after DCS drains the queues.
    double partsValid=(Aero.aeroForces.size()==7*Aero.rotorSubsteps && Aero.aeroMoments.size()==3*Aero.rotorSubsteps)?1:0;
    double partMx[3]={},partMy[3]={},partMz[3]={},forwardForce=0,rightForce=0;
    for(size_t i=0;i<Aero.aeroForces.size();++i) {
        const auto& f=Aero.aeroForces[i];
        forwardForce+=f.dir.x;rightForce+=f.dir.z;
        if(partsValid) {
            const size_t withinStep=i%7;
            const size_t group=withinStep<3?0:(withinStep<6?1:2);
            const double x=f.pos.x-EFMdata.centerOfGravity.x;
            const double y=f.pos.y-EFMdata.centerOfGravity.y;
            const double z=f.pos.z-EFMdata.centerOfGravity.z;
            partMx[group]+=y*f.dir.z-z*f.dir.y;
            partMy[group]+=z*f.dir.x-x*f.dir.z;
            partMz[group]+=x*f.dir.y-y*f.dir.x;
        }
    }
    if(partsValid)for(size_t i=0;i<Aero.aeroMoments.size();++i) {
        const size_t group=i%3;
        partMx[group]+=Aero.aeroMoments[i].x;
        partMy[group]+=Aero.aeroMoments[i].y;
        partMz[group]+=Aero.aeroMoments[i].z;
    }
    const auto frontState=Aero.debugRotorState(false),aftState=Aero.debugRotorState(true);
    const double bodySlip=EFMdata.totalVelocity_MPS>1e-6 ?
        std::asin(std::max(-1.0,std::min(1.0,EFMdata.airspeed.y/EFMdata.totalVelocity_MPS)))*Convert::radToDeg : 0.0;
    static void* h_TIME=api.getParamHandle("CH46_DBG_TIME"); // s
    api.setParamNumber(h_TIME, EFMdata.time);
    static void* h_DT=api.getParamHandle("CH46_DBG_DT"); // s
    api.setParamNumber(h_DT, dt);
    static void* h_COLLECTIVE=api.getParamHandle("CH46_DBG_COLLECTIVE"); // 0..1
    api.setParamNumber(h_COLLECTIVE, flightControls.CollectiveInput);
    static void* h_ROLL_RAW=api.getParamHandle("CH46_DBG_ROLL_RAW"); // -1..1
    api.setParamNumber(h_ROLL_RAW, flightControls.RollInput);
    static void* h_PITCH_RAW=api.getParamHandle("CH46_DBG_PITCH_RAW"); // -1..1
    api.setParamNumber(h_PITCH_RAW, flightControls.PitchInput);
    static void* h_PEDAL=api.getParamHandle("CH46_DBG_PEDAL"); // -1..1
    api.setParamNumber(h_PEDAL, flightControls.PedalInput);
    static void* h_ROLL_OUT=api.getParamHandle("CH46_DBG_ROLL_OUT"); // -1..1 after trim
    api.setParamNumber(h_ROLL_OUT, flightControls.rollOutput);
    static void* h_PITCH_OUT=api.getParamHandle("CH46_DBG_PITCH_OUT"); // -1..1 after trim
    api.setParamNumber(h_PITCH_OUT, flightControls.pitchOutput);
    static void* h_ROLL_TRIM=api.getParamHandle("CH46_DBG_ROLL_TRIM"); // normalized
    api.setParamNumber(h_ROLL_TRIM, flightControls.rollTrim);
    static void* h_PITCH_TRIM=api.getParamHandle("CH46_DBG_PITCH_TRIM"); // normalized
    api.setParamNumber(h_PITCH_TRIM, flightControls.pitchTrim);
    static void* h_ROTOR_SUBSTEPS=api.getParamHandle("CH46_DBG_ROTOR_SUBSTEPS");
    api.setParamNumber(h_ROTOR_SUBSTEPS,Aero.rotorSubsteps);
    static void* h_ROTOR_DT=api.getParamHandle("CH46_DBG_ROTOR_DT");
    api.setParamNumber(h_ROTOR_DT,Aero.rotorStepSeconds);
    static void* h_IDLE_STOP=api.getParamHandle("CH46_DBG_IDLE_STOP_RELEASED");
    api.setParamNumber(h_IDLE_STOP,throttleControl.idleStopReleased()?1.0:0.0);
    static void* h_THROTTLE=api.getParamHandle("CH46_DBG_THROTTLE"); // normalized
    api.setParamNumber(h_THROTTLE, Engine.throttleInput);
    static void* h_NR=api.getParamHandle("CH46_DBG_NR"); // rpm
    api.setParamNumber(h_NR, Aero.getMRomega()*Convert::radSecToRPM);
    static void* h_N2=api.getParamHandle("CH46_DBG_N2"); // percent
    api.setParamNumber(h_N2, Aero.getN2PCT());
    static void* h_N1=api.getParamHandle("CH46_DBG_N1"); // percent
    api.setParamNumber(h_N1, Engine.getN1RPM()*100.0); // 0.1.2: ratio -> percent
    static void* h_ENGINE_TORQUE=api.getParamHandle("CH46_DBG_ENGINE_TORQUE"); // Nm aggregate commanded
    api.setParamNumber(h_ENGINE_TORQUE, Engine.getEngTorque()*Convert::lbft_to_Nm);
    static void* h_SHAFT_POWER=api.getParamHandle("CH46_DBG_SHAFT_POWER"); // hp torque times current N2 speed; not delivered clutch power
    api.setParamNumber(h_SHAFT_POWER, Engine.getEngTorque()*Aero.getN2omega()/550.0);
    static void* h_FUEL=api.getParamHandle("CH46_DBG_FUEL"); // kg EFM tracked
    api.setParamNumber(h_FUEL, Fuel.getInternalFuel());
    static void* h_FUEL_FLOW=api.getParamHandle("CH46_DBG_FUEL_FLOW"); // kg/h
    api.setParamNumber(h_FUEL_FLOW, Engine.getFuelFlow());
    static void* h_FUEL_ENABLED=api.getParamHandle("CH46_DBG_FUEL_ENABLED"); // boolean
    api.setParamNumber(h_FUEL_ENABLED, Fuel.isFuelFlow ? 1.0 : 0.0);
    static void* h_TOT=api.getParamHandle("CH46_DBG_TOT"); // C inherited estimate
    api.setParamNumber(h_TOT, Engine.getTOT());
    static void* h_MASS=api.getParamHandle("CH46_DBG_MASS"); // kg callback mass
    api.setParamNumber(h_MASS, EFMdata.mass_kg);
    static void* h_WEIGHT=api.getParamHandle("CH46_DBG_WEIGHT"); // N callback mass times g
    api.setParamNumber(h_WEIGHT, EFMdata.weight_N);
    static void* h_DENSITY=api.getParamHandle("CH46_DBG_DENSITY"); // kg/m3
    api.setParamNumber(h_DENSITY, EFMdata.rho);
    static void* h_AGL=api.getParamHandle("CH46_DBG_AGL"); // m reference point
    api.setParamNumber(h_AGL, EFMdata.altitudeAGL);
    static void* h_TAS=api.getParamHandle("CH46_DBG_TAS"); // m/s
    api.setParamNumber(h_TAS, EFMdata.totalVelocity_MPS);
    static void* h_FRONT_THRUST=api.getParamHandle("CH46_DBG_FRONT_THRUST"); // N rotor solver thrust
    api.setParamNumber(h_FRONT_THRUST, Aero.frontThrust());
    static void* h_AFT_THRUST=api.getParamHandle("CH46_DBG_AFT_THRUST"); // N rotor solver thrust
    api.setParamNumber(h_AFT_THRUST, Aero.aftThrust());
    static void* h_FRONT_LOAD=api.getParamHandle("CH46_DBG_FRONT_LOAD"); // Nm signed
    api.setParamNumber(h_FRONT_LOAD, Aero.frontLoadTorque()*Convert::lbft_to_Nm);
    static void* h_AFT_LOAD=api.getParamHandle("CH46_DBG_AFT_LOAD"); // Nm signed
    api.setParamNumber(h_AFT_LOAD, Aero.aftLoadTorque()*Convert::lbft_to_Nm);
    static void* h_BODY_UP_FORCE=api.getParamHandle("CH46_DBG_BODY_UP_FORCE"); // N total exported aero force body Y
    api.setParamNumber(h_BODY_UP_FORCE, fy);
    static void* h_UP_WEIGHT_RATIO=api.getParamHandle("CH46_DBG_UP_WEIGHT_RATIO"); // body Y force/weight; not world vertical lift; -1 invalid
    api.setParamNumber(h_UP_WEIGHT_RATIO, EFMdata.weight_N>0 ? fy/EFMdata.weight_N : -1.0);
    static void* h_ROLL_MOMENT=api.getParamHandle("CH46_DBG_ROLL_MOMENT"); // Nm includes force lever arms about current CG
    api.setParamNumber(h_ROLL_MOMENT, mx);
    static void* h_PITCH_MOMENT=api.getParamHandle("CH46_DBG_PITCH_MOMENT"); // Nm DCS Z
    api.setParamNumber(h_PITCH_MOMENT, mz);
    static void* h_YAW_MOMENT=api.getParamHandle("CH46_DBG_YAW_MOMENT"); // Nm DCS Y
    api.setParamNumber(h_YAW_MOMENT, my);
    static void* h_ROLL=api.getParamHandle("CH46_DBG_ROLL"); // degrees DCS
    api.setParamNumber(h_ROLL, dbgRoll);
    static void* h_PITCH=api.getParamHandle("CH46_DBG_PITCH"); // degrees DCS
    api.setParamNumber(h_PITCH, dbgPitch);
    static void* h_ROLL_RATE=api.getParamHandle("CH46_DBG_ROLL_RATE"); // degrees/s DCS X
    api.setParamNumber(h_ROLL_RATE, dbgP);
    static void* h_PITCH_RATE=api.getParamHandle("CH46_DBG_PITCH_RATE"); // degrees/s DCS Z
    api.setParamNumber(h_PITCH_RATE, dbgQ);
    static void* h_YAW_RATE=api.getParamHandle("CH46_DBG_YAW_RATE"); // degrees/s DCS Y
    api.setParamNumber(h_YAW_RATE, dbgR);
    static void* h_CLIMB=api.getParamHandle("CH46_DBG_CLIMB"); // m/s world Y
    api.setParamNumber(h_CLIMB, dbgClimb);
    static void* h_CG_X=api.getParamHandle("CH46_DBG_CG_X"); // m
    api.setParamNumber(h_CG_X, EFMdata.centerOfGravity.x);
    static void* h_CG_Y=api.getParamHandle("CH46_DBG_CG_Y"); // m
    api.setParamNumber(h_CG_Y, EFMdata.centerOfGravity.y);
    static void* h_CG_Z=api.getParamHandle("CH46_DBG_CG_Z"); // m
    api.setParamNumber(h_CG_Z, EFMdata.centerOfGravity.z);
    static void* h_INERTIA_ROLL=api.getParamHandle("CH46_DBG_INERTIA_ROLL"); // kg m2
    api.setParamNumber(h_INERTIA_ROLL, EFMdata.inertia.x);
    static void* h_ROLL_FEEDBACK=api.getParamHandle("CH46_DBG_ROLL_FEEDBACK"); // normalized, limited to +/-0.15
    api.setParamNumber(h_ROLL_FEEDBACK,Aero.rollRateFeedback);
    static void* h_YAW_FEEDBACK=api.getParamHandle("CH46_DBG_YAW_FEEDBACK"); // normalized, limited to +/-0.20
    api.setParamNumber(h_YAW_FEEDBACK,Aero.yawRateFeedback);
    static void* h_ROLL_MIX=api.getParamHandle("CH46_DBG_ROLL_MIX"); // normalized after feedback/clamp
    api.setParamNumber(h_ROLL_MIX,Aero.mixedRollInput);
    static void* h_PEDAL_MIX=api.getParamHandle("CH46_DBG_PEDAL_MIX"); // normalized after feedback/clamp
    api.setParamNumber(h_PEDAL_MIX,Aero.mixedPedalInput);
    // DBG-09: actual FCS-02 correction and final pitch mixer input. Existing
    // PITCH_RATE remains BODY rate, and PITCH_OUT remains pilot plus trim.
    static void* h_PITCH_ANGLE_RATE=api.getParamHandle("CH46_DBG_PITCH_ANGLE_RATE");
    api.setParamNumber(h_PITCH_ANGLE_RATE,Aero.pitchAngleRate*Convert::radToDeg);
    static void* h_PITCH_FEEDBACK=api.getParamHandle("CH46_DBG_PITCH_FEEDBACK");
    api.setParamNumber(h_PITCH_FEEDBACK,Aero.pitchRateFeedback);
    static void* h_PITCH_MIX=api.getParamHandle("CH46_DBG_PITCH_MIX");
    api.setParamNumber(h_PITCH_MIX,Aero.mixedPitchInput);
    static void* h_PITCH_LIMIT=api.getParamHandle("CH46_DBG_PITCH_LIMIT");
#ifdef CH46_RATE_DAMPING_OFF
    api.setParamNumber(h_PITCH_LIMIT,0.0);
#else
    api.setParamNumber(h_PITCH_LIMIT,0.15);
#endif
    static void* h_AIR_FORWARD=api.getParamHandle("CH46_DBG_AIR_FORWARD"); // m/s body forward
    api.setParamNumber(h_AIR_FORWARD,EFMdata.airspeed.x);
    static void* h_AIR_RIGHT=api.getParamHandle("CH46_DBG_AIR_RIGHT"); // m/s body right
    api.setParamNumber(h_AIR_RIGHT,EFMdata.airspeed.y);
    static void* h_AIR_DOWN=api.getParamHandle("CH46_DBG_AIR_DOWN"); // m/s body down
    api.setParamNumber(h_AIR_DOWN,EFMdata.airspeed.z);
#ifdef CH46_RATE_DAMPING_OFF
    const double mode=0;
#else
    const double mode=1;
#endif
    static void* h_MODE=api.getParamHandle("CH46_DBG_DAMPING_MODE");
    api.setParamNumber(h_MODE,mode);
    // DBG-06: appended fields preserve all original 52 CSV positions.
    static void* h_RUN_ID=api.getParamHandle("CH46_DBG_RUN_ID"); // count per DLL lifetime
    api.setParamNumber(h_RUN_ID,dbgRunId);
    static void* h_RUN_TIME=api.getParamHandle("CH46_DBG_RUN_TIME"); // s since last start callback
    api.setParamNumber(h_RUN_TIME,EFMdata.time-dbgRunStart);
    static void* h_BODY_AX=api.getParamHandle("CH46_DBG_BODY_AX"); // m/s2 DCS body X callback
    api.setParamNumber(h_BODY_AX,dbgAx);
    static void* h_BODY_AY=api.getParamHandle("CH46_DBG_BODY_AY"); // m/s2 DCS body Y callback
    api.setParamNumber(h_BODY_AY,dbgAy);
    static void* h_BODY_AZ=api.getParamHandle("CH46_DBG_BODY_AZ"); // m/s2 DCS body Z callback
    api.setParamNumber(h_BODY_AZ,dbgAz);
    static void* h_ROLL_ACCEL=api.getParamHandle("CH46_DBG_ROLL_ACCEL"); // deg/s2 DCS X callback
    api.setParamNumber(h_ROLL_ACCEL,dbgPdot);
    static void* h_PITCH_ACCEL=api.getParamHandle("CH46_DBG_PITCH_ACCEL"); // deg/s2 DCS Z callback
    api.setParamNumber(h_PITCH_ACCEL,dbgQdot);
    static void* h_YAW_ACCEL=api.getParamHandle("CH46_DBG_YAW_ACCEL"); // deg/s2 DCS Y callback
    api.setParamNumber(h_YAW_ACCEL,dbgRdot);
    static void* h_BODY_FORWARD_FORCE=api.getParamHandle("CH46_DBG_BODY_FORWARD_FORCE"); // N total exported body X
    api.setParamNumber(h_BODY_FORWARD_FORCE,forwardForce);
    static void* h_BODY_RIGHT_FORCE=api.getParamHandle("CH46_DBG_BODY_RIGHT_FORCE"); // N total exported body Z
    api.setParamNumber(h_BODY_RIGHT_FORCE,rightForce);
    static void* h_BODY_SIDESLIP=api.getParamHandle("CH46_DBG_BODY_SIDESLIP"); // deg asin(right airspeed/TAS); zero at rest
    api.setParamNumber(h_BODY_SIDESLIP,bodySlip);
    static void* h_PARTS_VALID=api.getParamHandle("CH46_DBG_PARTS_VALID"); // 0 or 1; validate moment decomposition
    api.setParamNumber(h_PARTS_VALID,partsValid);
    static void* h_FRONT_ROLL_MOMENT=api.getParamHandle("CH46_DBG_FRONT_ROLL_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_FRONT_ROLL_MOMENT,partMx[0]);
    static void* h_AFT_ROLL_MOMENT=api.getParamHandle("CH46_DBG_AFT_ROLL_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_AFT_ROLL_MOMENT,partMx[1]);
    static void* h_FUSE_ROLL_MOMENT=api.getParamHandle("CH46_DBG_FUSE_ROLL_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_FUSE_ROLL_MOMENT,partMx[2]);
    static void* h_FRONT_YAW_MOMENT=api.getParamHandle("CH46_DBG_FRONT_YAW_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_FRONT_YAW_MOMENT,partMy[0]);
    static void* h_AFT_YAW_MOMENT=api.getParamHandle("CH46_DBG_AFT_YAW_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_AFT_YAW_MOMENT,partMy[1]);
    static void* h_FUSE_YAW_MOMENT=api.getParamHandle("CH46_DBG_FUSE_YAW_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_FUSE_YAW_MOMENT,partMy[2]);
    static void* h_FRONT_PITCH_MOMENT=api.getParamHandle("CH46_DBG_FRONT_PITCH_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_FRONT_PITCH_MOMENT,partMz[0]);
    static void* h_AFT_PITCH_MOMENT=api.getParamHandle("CH46_DBG_AFT_PITCH_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_AFT_PITCH_MOMENT,partMz[1]);
    static void* h_FUSE_PITCH_MOMENT=api.getParamHandle("CH46_DBG_FUSE_PITCH_MOMENT"); // Nm about current CG; valid only if PARTS_VALID=1
    api.setParamNumber(h_FUSE_PITCH_MOMENT,partMz[2]);
    static void* h_FRONT_FLAP_MAX=api.getParamHandle("CH46_DBG_FRONT_FLAP_MAX"); // deg maximum absolute blade flap
    api.setParamNumber(h_FRONT_FLAP_MAX,frontState.maxFlapDeg);
    static void* h_FRONT_FLAP_LIMIT=api.getParamHandle("CH46_DBG_FRONT_FLAP_LIMIT"); // number of blades at flap bound
    api.setParamNumber(h_FRONT_FLAP_LIMIT,frontState.flapLimitBlades);
    static void* h_FRONT_INFLOW=api.getParamHandle("CH46_DBG_FRONT_INFLOW"); // normalized net rotor inflow
    api.setParamNumber(h_FRONT_INFLOW,frontState.inflow);
    static void* h_FRONT_DOWNWASH=api.getParamHandle("CH46_DBG_FRONT_DOWNWASH"); // normalized induced velocity
    api.setParamNumber(h_FRONT_DOWNWASH,frontState.downwash);
    static void* h_FRONT_PHASE=api.getParamHandle("CH46_DBG_FRONT_PHASE"); // rad solver rotor azimuth
    api.setParamNumber(h_FRONT_PHASE,frontState.phaseRad);
    static void* h_FRONT_BLADE_HEALTH=api.getParamHandle("CH46_DBG_FRONT_BLADE_HEALTH"); // minimum modeled blade integrity; existing shared damage indices
    api.setParamNumber(h_FRONT_BLADE_HEALTH,frontState.minBladeHealth);
    static void* h_AFT_FLAP_MAX=api.getParamHandle("CH46_DBG_AFT_FLAP_MAX"); // deg maximum absolute blade flap
    api.setParamNumber(h_AFT_FLAP_MAX,aftState.maxFlapDeg);
    static void* h_AFT_FLAP_LIMIT=api.getParamHandle("CH46_DBG_AFT_FLAP_LIMIT"); // number of blades at flap bound
    api.setParamNumber(h_AFT_FLAP_LIMIT,aftState.flapLimitBlades);
    static void* h_AFT_INFLOW=api.getParamHandle("CH46_DBG_AFT_INFLOW"); // normalized net rotor inflow
    api.setParamNumber(h_AFT_INFLOW,aftState.inflow);
    static void* h_AFT_DOWNWASH=api.getParamHandle("CH46_DBG_AFT_DOWNWASH"); // normalized induced velocity
    api.setParamNumber(h_AFT_DOWNWASH,aftState.downwash);
    static void* h_AFT_PHASE=api.getParamHandle("CH46_DBG_AFT_PHASE"); // rad solver rotor azimuth
    api.setParamNumber(h_AFT_PHASE,aftState.phaseRad);
    static void* h_AFT_BLADE_HEALTH=api.getParamHandle("CH46_DBG_AFT_BLADE_HEALTH"); // minimum modeled blade integrity; existing shared damage indices
    api.setParamNumber(h_AFT_BLADE_HEALTH,aftState.minBladeHealth);
    static void* h_ROLL_LIMIT=api.getParamHandle("CH46_DBG_ROLL_LIMIT"); // 0 or 1; feedback at 0.15 bound
    api.setParamNumber(h_ROLL_LIMIT,std::abs(Aero.rollRateFeedback)>=0.15-1e-9?1.0:0.0);
    static void* h_YAW_LIMIT=api.getParamHandle("CH46_DBG_YAW_LIMIT"); // 0 or 1; feedback at 0.20 bound
    api.setParamNumber(h_YAW_LIMIT,std::abs(Aero.yawRateFeedback)>=0.20-1e-9?1.0:0.0);
    static void* h_DAMAGE_COUNT=api.getParamHandle("CH46_DBG_DAMAGE_COUNT"); // received callback count; not accepted damage count
    api.setParamNumber(h_DAMAGE_COUNT,dbgDamageCount);
    static void* h_DAMAGE_ELEMENT=api.getParamHandle("CH46_DBG_DAMAGE_ELEMENT"); // last received element; -1 none
    api.setParamNumber(h_DAMAGE_ELEMENT,dbgDamageElement);
    static void* h_DAMAGE_FACTOR=api.getParamHandle("CH46_DBG_DAMAGE_FACTOR"); // last received integrity factor
    api.setParamNumber(h_DAMAGE_FACTOR,dbgDamageFactor);
    static void* h_DAMAGE_TIME=api.getParamHandle("CH46_DBG_DAMAGE_TIME"); // EFM TIME of last callback; -1 none
    api.setParamNumber(h_DAMAGE_TIME,dbgDamageTime);
    // DBG-11: observations only; mathematical mixer headroom is not hardware travel.
    static void* h_POWER_AVAILABLE=api.getParamHandle("CH46_DBG_POWER_AVAILABLE"); // hp aggregate engine map budget
    api.setParamNumber(h_POWER_AVAILABLE, Engine.getAvailablePowerHP());
    static void* h_POWER_APPLIED=api.getParamHandle("CH46_DBG_POWER_APPLIED"); // hp mean pre-clutch power actually applied over substeps
    api.setParamNumber(h_POWER_APPLIED, Aero.appliedEnginePowerHP);
    static void* h_TORQUE_APPLIED=api.getParamHandle("CH46_DBG_TORQUE_APPLIED"); // Nm mean engine torque before internal loss/clutch
    api.setParamNumber(h_TORQUE_APPLIED, Aero.appliedEngineTorque*Convert::lbft_to_Nm);
    static void* h_CLUTCH_TORQUE=api.getParamHandle("CH46_DBG_CLUTCH_TORQUE"); // Nm mean nonnegative torque transferred to rotors
    api.setParamNumber(h_CLUTCH_TORQUE, Aero.clutchTorqueMean*Convert::lbft_to_Nm);
    static void* h_CLUTCH_ENGAGED=api.getParamHandle("CH46_DBG_CLUTCH_ENGAGED"); // fraction 0..1 of substeps transferring clutch impulse
    api.setParamNumber(h_CLUTCH_ENGAGED, Aero.clutchEngagedFraction);
    static void* h_NR_LIMIT_ACTIVE=api.getParamHandle("CH46_DBG_NR_LIMIT_ACTIVE"); // fraction 0..1 of substeps hitting numerical rotor ceiling
    api.setParamNumber(h_NR_LIMIT_ACTIVE, Aero.rotorSpeedLimitFraction);
    static void* h_ROLL_ROOM_NEG=api.getParamHandle("CH46_DBG_ROLL_ROOM_NEG"); // normalized distance to negative mixer limit; not joystick travel
    api.setParamNumber(h_ROLL_ROOM_NEG, 1.0+Aero.mixedRollInput);
    static void* h_ROLL_ROOM_POS=api.getParamHandle("CH46_DBG_ROLL_ROOM_POS"); // normalized distance to positive mixer limit; not joystick travel
    api.setParamNumber(h_ROLL_ROOM_POS, 1.0-Aero.mixedRollInput);
    static void* h_PITCH_ROOM_NEG=api.getParamHandle("CH46_DBG_PITCH_ROOM_NEG"); // normalized distance to aft mixer limit; not joystick travel
    api.setParamNumber(h_PITCH_ROOM_NEG, 1.0+Aero.mixedPitchInput);
    static void* h_PITCH_ROOM_POS=api.getParamHandle("CH46_DBG_PITCH_ROOM_POS"); // normalized distance to forward mixer limit; not joystick travel
    api.setParamNumber(h_PITCH_ROOM_POS, 1.0-Aero.mixedPitchInput);
    static void* h_PEDAL_ROOM_NEG=api.getParamHandle("CH46_DBG_PEDAL_ROOM_NEG"); // normalized distance to negative mixer limit; not joystick travel
    api.setParamNumber(h_PEDAL_ROOM_NEG, 1.0+Aero.mixedPedalInput);
    static void* h_PEDAL_ROOM_POS=api.getParamHandle("CH46_DBG_PEDAL_ROOM_POS"); // normalized distance to positive mixer limit; not joystick travel
    api.setParamNumber(h_PEDAL_ROOM_POS, 1.0-Aero.mixedPedalInput);
    static void* h_BLADE_FIRST_MOMENT=api.getParamHandle("CH46_DBG_BLADE_FIRST_MOMENT"); // slug-ft provisional derived per-blade first mass moment
    api.setParamNumber(h_BLADE_FIRST_MOMENT, CH46::bladeFirstMassMoment);
    static void* h_ROTOR_INERTIA=api.getParamHandle("CH46_DBG_ROTOR_INERTIA"); // slug-ft^2 provisional blade-only inertia of both rotors
    api.setParamNumber(h_ROTOR_INERTIA, CH46::tandemBladeInertia);
    // DBG-12: append actual SDK outputs and start/vertical-force observations.
    static void* h_ENGINE_STATE=api.getParamHandle("CH46_DBG_ENGINE_STATE"); // 0 off, 1 motor, 2 ignition, 3 running; shared engine model
    api.setParamNumber(h_ENGINE_STATE, Engine.stateForDebug());
    static void* h_STARTER_SWITCH=api.getParamHandle("CH46_DBG_STARTER_SWITCH"); // commanded starter button
    api.setParamNumber(h_STARTER_SWITCH, Engine.starterButtonOn?1.0:0.0);
    static void* h_STARTER_ENGAGED=api.getParamHandle("CH46_DBG_STARTER_ENGAGED"); // existing engine starter state; not a measured torque
    api.setParamNumber(h_STARTER_ENGAGED, Engine.isStarterEngaged()?1.0:0.0);
    static void* h_APU_TORQUE=api.getParamHandle("CH46_DBG_APU_TORQUE"); // Nm SDK APU output, zero while APU is unmodeled
    api.setParamNumber(h_APU_TORQUE, ed_fm_get_param(ED_FM_ENGINE_0_TORQUE));
    static void* h_STARTER1_TORQUE=api.getParamHandle("CH46_DBG_STARTER1_TORQUE"); // SDK starter-related torque, unmodeled and zero
    api.setParamNumber(h_STARTER1_TORQUE, ed_fm_get_param(ED_FM_ENGINE_1_STARTER_RELATED_TORQUE));
    static void* h_STARTER2_TORQUE=api.getParamHandle("CH46_DBG_STARTER2_TORQUE"); // SDK starter-related torque, unmodeled and zero
    api.setParamNumber(h_STARTER2_TORQUE, ed_fm_get_param(ED_FM_ENGINE_2_STARTER_RELATED_TORQUE));
    static void* h_ENGINE1_TORQUE=api.getParamHandle("CH46_DBG_ENGINE1_TORQUE"); // Nm individual model engine-output-shaft torque
    api.setParamNumber(h_ENGINE1_TORQUE, ed_fm_get_param(ED_FM_ENGINE_1_TORQUE));
    static void* h_ENGINE2_TORQUE=api.getParamHandle("CH46_DBG_ENGINE2_TORQUE"); // Nm individual model engine-output-shaft torque
    api.setParamNumber(h_ENGINE2_TORQUE, ed_fm_get_param(ED_FM_ENGINE_2_TORQUE));
    static void* h_ENGINE1_FUEL_FLOW=api.getParamHandle("CH46_DBG_ENGINE1_FUEL_FLOW"); // kg/s half of aggregate consumption
    api.setParamNumber(h_ENGINE1_FUEL_FLOW, ed_fm_get_param(ED_FM_ENGINE_1_FUEL_FLOW));
    static void* h_ENGINE2_FUEL_FLOW=api.getParamHandle("CH46_DBG_ENGINE2_FUEL_FLOW"); // kg/s half of aggregate consumption
    api.setParamNumber(h_ENGINE2_FUEL_FLOW, ed_fm_get_param(ED_FM_ENGINE_2_FUEL_FLOW));
    static void* h_MODEL_TORQUE_PCT=api.getParamHandle("CH46_DBG_MODEL_TORQUE_PCT"); // percent of model reference torque, not NATOPS gauge calibration
    api.setParamNumber(h_MODEL_TORQUE_PCT, 100.0*Engine.getTorqueRelative());
    static void* h_WORLD_UP_WEIGHT_RATIO=api.getParamHandle("CH46_DBG_WORLD_UP_WEIGHT_RATIO"); // world-vertical aerodynamic force / weight, gravity excluded
    api.setParamNumber(h_WORLD_UP_WEIGHT_RATIO, EFMdata.weight_N>0?CH46::verticalOffset(forwardForce,fy,rightForce,dbgPitch*Convert::degToRad,dbgRoll*Convert::degToRad)/EFMdata.weight_N:-1.0);
    static void* h_FRONT_HUB_AGL=api.getParamHandle("CH46_DBG_FRONT_HUB_AGL"); // m vertical front hub height over the reference-point surface
    api.setParamNumber(h_FRONT_HUB_AGL, EFMdata.altitudeAGL+CH46::verticalOffset(CH46::frontHubX,CH46::frontHubY,0,dbgPitch*Convert::degToRad,dbgRoll*Convert::degToRad));
    static void* h_AFT_HUB_AGL=api.getParamHandle("CH46_DBG_AFT_HUB_AGL"); // m vertical aft hub height over the reference-point surface
    api.setParamNumber(h_AFT_HUB_AGL, EFMdata.altitudeAGL+CH46::verticalOffset(CH46::aftHubX,CH46::aftHubY,0,dbgPitch*Convert::degToRad,dbgRoll*Convert::degToRad));
    // DBG-13: diagnostic-only observations for drivetrain and governor checks.
    static void* h_FRONT_LOAD_MEAN=api.getParamHandle("CH46_DBG_FRONT_LOAD_MEAN"); // Nm host-frame mean rotor load
    api.setParamNumber(h_FRONT_LOAD_MEAN, Aero.frontLoadMean*Convert::lbft_to_Nm);
    static void* h_AFT_LOAD_MEAN=api.getParamHandle("CH46_DBG_AFT_LOAD_MEAN"); // Nm host-frame mean rotor load
    api.setParamNumber(h_AFT_LOAD_MEAN, Aero.aftLoadMean*Convert::lbft_to_Nm);
    static void* h_SHAFT_BALANCE_ERROR=api.getParamHandle("CH46_DBG_SHAFT_BALANCE_ERROR"); // Nm host-frame mean shaft momentum residual; numerical clamps included
    api.setParamNumber(h_SHAFT_BALANCE_ERROR, Aero.shaftBalanceErrorNm);
    static void* h_GOVERNOR_TARGET=api.getParamHandle("CH46_DBG_GOVERNOR_TARGET"); // percent N2 existing governor target; zero when not running
    api.setParamNumber(h_GOVERNOR_TARGET, Engine.getGovernorTargetN2Pct());
    static void* h_GOVERNOR_ERROR=api.getParamHandle("CH46_DBG_GOVERNOR_ERROR"); // percentage points target minus current N2; zero when not running
    api.setParamNumber(h_GOVERNOR_ERROR, Engine.getGovernorTargetN2Pct()>0?Engine.getGovernorTargetN2Pct()-Aero.getN2PCT():0.0);
    api.setParamNumber(version,111);
    api.setParamNumber(sequence,++seq); // published last: monitor detects a live producer
    // DBG-14: independent systems and virtual-gauge observations. No state advance.
    const auto ball=CH46::slipBall(dbgAy,dbgAz,dbgPitch*Convert::degToRad,dbgRoll*Convert::degToRad);
    static void* h_ENGINE1_STATE=api.getParamHandle("CH46_DBG_ENGINE1_STATE"); // 0 off / 1 motor / 2 ignition / 3 running
    api.setParamNumber(h_ENGINE1_STATE, Engine.unit(0).stateForDebug());
    static void* h_ENGINE1_N1=api.getParamHandle("CH46_DBG_ENGINE1_N1"); // percent
    api.setParamNumber(h_ENGINE1_N1, 100.0*Engine.unit(0).getN1RPM());
    static void* h_ENGINE1_POWER=api.getParamHandle("CH46_DBG_ENGINE1_POWER"); // hp available
    api.setParamNumber(h_ENGINE1_POWER, Engine.unit(0).getAvailablePowerHP());
    static void* h_ENGINE1_FEED=api.getParamHandle("CH46_DBG_ENGINE1_FEED"); // bool
    api.setParamNumber(h_ENGINE1_FEED, Fuel.hasEngineFuel(0));
    static void* h_ENGINE1_STARTER=api.getParamHandle("CH46_DBG_ENGINE1_STARTER"); // bool
    api.setParamNumber(h_ENGINE1_STARTER, Engine.unit(0).isStarterEngaged());
    static void* h_ENGINE2_STATE=api.getParamHandle("CH46_DBG_ENGINE2_STATE"); // 0 off / 1 motor / 2 ignition / 3 running
    api.setParamNumber(h_ENGINE2_STATE, Engine.unit(1).stateForDebug());
    static void* h_ENGINE2_N1=api.getParamHandle("CH46_DBG_ENGINE2_N1"); // percent
    api.setParamNumber(h_ENGINE2_N1, 100.0*Engine.unit(1).getN1RPM());
    static void* h_ENGINE2_POWER=api.getParamHandle("CH46_DBG_ENGINE2_POWER"); // hp available
    api.setParamNumber(h_ENGINE2_POWER, Engine.unit(1).getAvailablePowerHP());
    static void* h_ENGINE2_FEED=api.getParamHandle("CH46_DBG_ENGINE2_FEED"); // bool
    api.setParamNumber(h_ENGINE2_FEED, Fuel.hasEngineFuel(1));
    static void* h_ENGINE2_STARTER=api.getParamHandle("CH46_DBG_ENGINE2_STARTER"); // bool
    api.setParamNumber(h_ENGINE2_STARTER, Engine.unit(1).isStarterEngaged());
    static void* h_FUEL_LEFT=api.getParamHandle("CH46_DBG_FUEL_LEFT"); // kg
    api.setParamNumber(h_FUEL_LEFT, Fuel.tankFuel(0));
    static void* h_FUEL_RIGHT=api.getParamHandle("CH46_DBG_FUEL_RIGHT"); // kg
    api.setParamNumber(h_FUEL_RIGHT, Fuel.tankFuel(1));
    static void* h_CROSSFEED=api.getParamHandle("CH46_DBG_CROSSFEED"); // bool
    api.setParamNumber(h_CROSSFEED, Fuel.crossfeedOpen());
    static void* h_FUEL_VALVE1=api.getParamHandle("CH46_DBG_FUEL_VALVE1"); // bool
    api.setParamNumber(h_FUEL_VALVE1, Fuel.valveOpen(0));
    static void* h_FUEL_VALVE2=api.getParamHandle("CH46_DBG_FUEL_VALVE2"); // bool
    api.setParamNumber(h_FUEL_VALVE2, Fuel.valveOpen(1));
    static void* h_FUEL_LOW_LEFT=api.getParamHandle("CH46_DBG_FUEL_LOW_LEFT"); // bool
    api.setParamNumber(h_FUEL_LOW_LEFT, Fuel.tankLow(0));
    static void* h_FUEL_LOW_RIGHT=api.getParamHandle("CH46_DBG_FUEL_LOW_RIGHT"); // bool
    api.setParamNumber(h_FUEL_LOW_RIGHT, Fuel.tankLow(1));
    static void* h_ALTITUDE_MSL=api.getParamHandle("CH46_DBG_ALTITUDE_MSL"); // m geometric, not pressure altitude
    api.setParamNumber(h_ALTITUDE_MSL, EFMdata.altitudeMSL);
    static void* h_SLIP_BALL=api.getParamHandle("CH46_DBG_SLIP_BALL"); // degrees apparent gravity deflection
    api.setParamNumber(h_SLIP_BALL, ball.degrees);
    static void* h_SLIP_BALL_VALID=api.getParamHandle("CH46_DBG_SLIP_BALL_VALID"); // bool
    api.setParamNumber(h_SLIP_BALL_VALID, ball.valid);
    static void* h_FRONT_DAMAGE_GROUP=api.getParamHandle("CH46_DBG_FRONT_DAMAGE_GROUP"); // DCS group cell ID or -1 unresolved
    api.setParamNumber(h_FRONT_DAMAGE_GROUP, damageModel.rotorGroupElement(false));
    static void* h_AFT_DAMAGE_GROUP=api.getParamHandle("CH46_DBG_AFT_DAMAGE_GROUP"); // DCS group cell ID or -1 unresolved
    api.setParamNumber(h_AFT_DAMAGE_GROUP, damageModel.rotorGroupElement(true));
    std::snprintf(dbgText,sizeof(dbgText),
      "CH46 DEBUG 0.1.11 TEST mode=%.0f t=%.1f | COL %.0f%% R/P/Y %.2f %.2f %.2f\n"
      "NR %.1f rpm N1/N2 %.1f/%.1f%% Fuel %.1f kg Mass %.0f kg\n"
      "Roll %.1f deg Rate %.1f deg/s Mx %.0f Nm | BodyUp/W %.2f\n"
      "Front/Aft %.0f/%.0f N | TAS %.1f m/s Climb %.1f m/s AGL %.1f m",
      mode,EFMdata.time,100*flightControls.CollectiveInput,flightControls.rollOutput,
      flightControls.pitchOutput,flightControls.PedalInput,
      Aero.getMRomega()*Convert::radSecToRPM,Engine.getN1RPM()*100.0,Aero.getN2PCT(),
      Fuel.getInternalFuel(),EFMdata.mass_kg,dbgRoll,dbgP,mx,
      EFMdata.weight_N>0?fy/EFMdata.weight_N:-1.0,Aero.frontThrust(),Aero.aftThrust(),
      EFMdata.totalVelocity_MPS,dbgClimb,EFMdata.altitudeAGL);
}
