#pragma once
#include "../FlightModel/TandemConfig.h"
#include <cmath>
#include <vector>
 

// TODO: transfer of fuel between tanks (need handle added in 3d model)

class FuelTank
{
private:
	double capacity = 0.0; // max capacity of tank [kg]
	Vec3 position;// for weight and balance
public:	
	double currentFuel = 0.0; // amount of fuel currently in tank [kg]

	FuelTank(double maxCapacity = 0, double posX = 0, double posY = 0, double posZ = 0)
		: capacity(maxCapacity)
		, position(posX, posY, posZ)
	{}
	~FuelTank() {}

	// add what is possible, return remaining if full
	double addFuel(const double addition)
	{
		double space = capacity - currentFuel;
		if (space < addition)
		{
			currentFuel = capacity; // set to max
			return (addition - space); // overflow
		}
		currentFuel += addition;
		return 0.0;
	}

	double decFuel(const double decrement)
	{
		if (currentFuel < decrement)
		{
            // FUEL-01: preserve available fuel BEFORE emptying the tank.
            // Example: 0.5 kg available, 1 kg requested -> 0.5 kg still needed.
            // Subtracting after setting currentFuel=0 passed the full request
            // onward and burned the last 0.5 kg twice across the tank boundary.
            const double available = currentFuel;
            currentFuel = 0.0;
            return decrement - available;
		}
		currentFuel -= decrement;
		return 0.0;
	}

};

// FUEL-05: left/right cells normally feed their corresponding engines.
// The supplied H-46D manual sections 2.5.2.2 and 2.5.2.6 describe crossfeed
// and separate low-level indications. No pump-pressure or tank-location/CG
// model is claimed; both mass deltas remain at the existing model origin.
class FuelSystem {
    FuelTank tanks_[2]{{CH46::fuelReservoirCapacityKg},{CH46::fuelReservoirCapacityKg}};
    bool isUnlimitedFuel=false,isIdleCutoff=false,crossfeed_=false;
    bool valve_[2]{true,true};
    void refresh(){isFuelFlow=hasEngineFuel(0)||hasEngineFuel(1);}
    void init(bool cold){
        fuelMassDelta.clear();isIdleCutoff=cold;crossfeed_=false;
        valve_[0]=valve_[1]=true;
        for(auto& tank:tanks_)if(tank.currentFuel>0)fuelMassDelta.push_back(tank.currentFuel);
        refresh();
    }
public:
    bool isFuelFlow=false;
    std::vector<double> fuelMassDelta;
    void initCold(){init(true);} void initHot(){init(false);}
    double tankFuel(int i)const{return i>=0&&i<2?tanks_[i].currentFuel:0;}
    bool valveOpen(int i)const{return i>=0&&i<2&&valve_[i];}
    bool crossfeedOpen()const{return crossfeed_;}
    bool hasEngineFuel(int i)const{
        return valveOpen(i)&&!isIdleCutoff&&(tankFuel(i)>0||(crossfeed_&&getInternalFuel()>0));
    }
    void setEngineValve(int i,bool open){if(i>=0&&i<2)valve_[i]=open;refresh();}
    void setCrossfeed(bool open){crossfeed_=open;refresh();}
    // 1997 H-46D quantity warning: about 25 percent. An approximation to the
    // level switch, not attitude-dependent sensing or verified Vietnam wiring.
    bool tankLow(int i)const{return tankFuel(i)<=.25*CH46::fuelReservoirCapacityKg;}
    bool isLowFuel()const{return tankLow(0)||tankLow(1);}
    double getInternalFuel()const{return tanks_[0].currentFuel+tanks_[1].currentFuel;}
    void setInternalFuel(double kg){
        if(!std::isfinite(kg)||kg<0)return;
        const double total=std::min(kg,CH46::internalFuelCapacityKg);
        tanks_[0].currentFuel=tanks_[1].currentFuel=.5*total;refresh();
    }
    void refuelAdd(double kg){
        if(!std::isfinite(kg)||kg<=0)return;
        const double before=getInternalFuel();
        // Equal delivery where both cells have room; remaining accepted fuel
        // goes to whichever cell has space. No transfer between cells occurs.
        double left=tanks_[0].addFuel(.5*kg),right=tanks_[1].addFuel(.5*kg);
        if(left>0)tanks_[1].addFuel(left);
        if(right>0)tanks_[0].addFuel(right);
        const double accepted=getInternalFuel()-before;
        if(accepted>0)fuelMassDelta.push_back(accepted);refresh();
    }
    void setThrottle(float value){if(std::isfinite(value))isIdleCutoff=value<-.2;refresh();}
    void setUnlimitedFuel(bool on){isUnlimitedFuel=on;}
    void update(double ff1,double ff2,double dt){
        if(!std::isfinite(dt)||dt<0)return;
        const double before=getInternalFuel();
        double demand[2]{ff1,ff2},unmet[2]{0,0};
        for(int i=0;i<2;++i){
            const double amount=!isUnlimitedFuel&&hasEngineFuel(i)&&std::isfinite(demand[i])?
                std::max(0.0,demand[i])*dt/3600:0;
            unmet[i]=tanks_[i].decFuel(amount);
        }
        // Idealized crossfeed: each engine uses its own cell first; an open
        // crossfeed permits unmet demand from the other cell. Hydraulic flow
        // sharing, boost-pump switches and altitude effects remain unmodeled.
        if(crossfeed_){tanks_[1].decFuel(unmet[0]);tanks_[0].decFuel(unmet[1]);}
        const double burn=before-getInternalFuel();
        if(burn>0)fuelMassDelta.push_back(-burn);
        refresh();G_Params.cautionLight[CL_FuelLow]=isLowFuel();
    }
    void update(double totalFF,double dt){update(.5*totalFF,.5*totalFF,dt);}
};
