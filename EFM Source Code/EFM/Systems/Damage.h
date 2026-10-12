#pragma once
#include <algorithm>
#include <cmath>

class AH6JDamage
{
private:

	// damage status of section in 0.01 increments. 0 is fully broken, 1 is full health
	static const int maxElem = 137;// max number of damage elements from Scripts\Aircrafts\_Common\Damage.lua
	int rotorGroup_[2]{-1,-1}; // DMG-01: resolved from the installed DCS Damage.lua, never guessed.
	bool isImmortal = false; // <- ignore damage

public:

	AH6JDamage() {}
	~AH6JDamage() {}

    // DMG-01: six independent center channels. The original model only
    // declares rotor groups, so its separate-blade collision support is pending.
    int rotorGroupElement(bool aft)const{return rotorGroup_[aft?1:0];}
    void setRotorGroupElement(bool aft,double value){
        if(std::isfinite(value)&&value==std::floor(value)&&value>=0&&value<maxElem)
            rotorGroup_[aft?1:0]=static_cast<int>(value);
    }
    double bladeHealth(bool aft,int blade)const{
        if(blade<0||blade>=3)return 1.0;
        const int cell=BLADE_1_CENTER+(blade+(aft?3:0))*3;
        const int group=rotorGroup_[aft?1:0];
        const double individual=elementIntegrity[cell];
        return group>=0?std::min(individual,elementIntegrity[group]):individual;
    }
	double elementIntegrity[maxElem]{ 0.0 }; // store each damage element 


	void init()
	{
		for (int i = 0; i < maxElem; i++)
		{
			elementIntegrity[i] = 1.0;
		}
	}


	// Scripts\Aircrafts\_Common\Damage.lua	 for element numbers
	void onAirframeDamage(int element, double elementIntegrityFactor)
	{
		if (element >= 0 && element < maxElem && !isImmortal && std::isfinite(elementIntegrityFactor))
		{
			elementIntegrity[element] = std::clamp(elementIntegrityFactor,0.0,1.0);
		}
	}

	void onRepair()
	{
		for (int i = 0; i < maxElem; i++)
		{
			elementIntegrity[i] = 1.0;
		}
	}

	bool isRepairNeeded() const
	{
		// zero is fully broken
		for (int i = 0; i < maxElem; i++)
		{
			if (elementIntegrity[i] < 1)
			{
				return true;
			}
		}
		return false;
	}

	void setImmortal(bool value)
	{
		isImmortal = value;
	}

};
