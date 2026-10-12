#pragma once
#include <algorithm>
#include <cmath>

// CTRL-03/04: command routing, not a researched CH-46D engine-condition lever.
// Supplied EFM_Helper.lua uses cutoff argument 155 as an idle-stop release:
// 0 locks travel below idle; 1 permits it; positive throttle relocks the stop.
// Actual fuel cutoff remains the existing throttle < -0.2 convention.
class ThrottleControl {
    bool idleStopReleased_=false;
public:
    void reset(bool cold){idleStopReleased_=cold;}
    void setIdleStopRelease(double value){
        if(std::isfinite(value))idleStopReleased_=value>0.5;
    }
    bool idleStopReleased()const{return idleStopReleased_;}
    template<class EngineType,class FuelType>
    void clickable(double value,EngineType& engine,FuelType& fuel){
        if(!std::isfinite(value))return;
        value=std::clamp(value,-1.0,1.0);
        if(!idleStopReleased_)value=std::max(0.0,value);
        if(value>0.0)idleStopReleased_=false;
        engine.throttleInput=value;fuel.setThrottle(static_cast<float>(value));
    }
    template<class EngineType,class FuelType>
    void axis(double value,EngineType& engine,FuelType& fuel){
        if(!std::isfinite(value))return;
        // CTRL-03: 3016 is an ENGINE throttle axis; 2004 is collective.
        // Match the sample convention: -1 full, +1 idle. An axis cannot
        // enter the separate negative clickable cutoff travel accidentally.
        clickable((1.0-std::clamp(value,-1.0,1.0))*.5,engine,fuel);
    }
};
