#pragma once
#include "../FlightModel/Aero.h"


// N1 gas producer rpm will be main measure of engine power
// N2 is handled in the rotor degree of freedom section in Aero.cpp since it is connected to the rotor through the clutch
//
//TODO: accurate fuel flow

// TODO validate numbers (oil temp, oil press, TOT, fuel flow...)
// fig 7-30 (pg.7-17) idle fuel flow for 65%N2 (64%N1 true idle) and 100%N2 (throttle is open, N1~=75% estimate)
// fig 7-33 (pg.7-19) TOT based on torque, and torque-horsepower conversion

// engine data tables
constexpr static const int _N1_Points = 9;
static double _N1percent[_N1_Points] = { 0.0, 20.0,  62.0,  80.0,  95.0,  98.0,  100.0,  105.0, 106.0};
// new array for CH-46D
// static double _HP_data[_N1_Points] =   { 0.0,  0.0,  45.0, 135.0, 312.0, 370.0,  400.0,  425.0, 575.0};
// static double _HP_data[_N1_Points] =   { 0.0, 0.0, 420.0, 1150.0, 2200.0, 2550.0, 2800.0, 2950.0, 3200.0};
// Provisional AGGREGATE map. Shape is inherited; only the 2 x 1400 shp
// ceiling is source-backed. Independent T58s and transmission limits remain TODO.
static double _HP_data[_N1_Points] =   { 0.0, 0.0, 420.0, 1150.0, 2200.0, 2550.0, 2800.0, 2800.0, 2800.0};

class TurboshaftEngine
{
private:

    // ENG-01: share=0.5 is one of two T58 states. The inherited map is still aggregate.
    const double powerShare_;
    const bool publishCockpit_;
    EFMData& p_EFMdata;
    FlightControls& p_flightControl;

    enum engineStates
    {
        ENG_Off,
        ENG_Motor,
        ENG_Ignition,
        ENG_Running
    } engineState = ENG_Off;

    constexpr static double ENG_RPM_MOTOR_MIN_PCT = 12.0;
    constexpr static double ENG_RPM_MOTOR_PCT = 19.0;
    constexpr static double ENG_RPM_SUSTAIN_PCT = 58.0;
    constexpr static double ENG_RPM_IDLE_PCT = 64.0;
    constexpr static double ENG_RPM_MAX_LIMIT_PCT = 106.0;

    MatrixFunction fn_engHP{ _HP_data, _N1_Points, _N1percent };

    EDPARAM cockpitAPI;
    void* ENGINE_TRQ = cockpitAPI.getParamHandle("ENGINE_TRQ"); // for use in digital VIDS gauge
    void* ENGINE_TOT = cockpitAPI.getParamHandle("ENGINE_TOT"); // for use in digital VIDS gauge

    double TargetN2 = 98.0;// adjustable governor target to be used for collective switch
    double governorTrimN1 = 0.0; // integral trim removes steady rotor-speed error
    double N1Cmd = 100.0;// target N1 before delta limit
    double N1rate = 15.0;
    double N1_PCT = 100.0;// current N1 speed [%] (gas producer speed)
    double fuelFlow = 0.0; // [lb/hr]
    double Torque_PSI = 0.0;
    double engTorque_ftlb = 0.0;
    double availablePowerHP = 0.0; // PWR-01 aggregate power budget before clutch
    double oilPressure = 0.0; // [PSI]
    double oilTemp = 0.0; // [deg C]
    double TOTcmd = 15.0; // [deg C]
    double TOTrate = 10.0;
    double turbineOutletTemp = 15.0; // current TOT [deg C]
    double addTemp = 0.0;
    double TOTwhenIgnited = 0.0;

public:
    double throttleInput = 0.0;	// Throttle input command
    bool starterButtonOn = false;

    TurboshaftEngine(EFMData& ptr_EFMdata, FlightControls& ptr_fltCntrl,
        double powerShare=1.0, bool publishCockpit=true)
        : powerShare_(powerShare), publishCockpit_(publishCockpit), p_EFMdata(ptr_EFMdata)
        , p_flightControl(ptr_fltCntrl)
    {}
    ~TurboshaftEngine() {}

    void initCold()
    {
        TOTrate = 10.0; addTemp = TOTwhenIgnited = 0.0; // RESET-03: complete thermal reset.
        engineState = ENG_Off;
        TargetN2 = 98.0;
        governorTrimN1 = 0.0;
        throttleInput = 0;
        starterButtonOn = false;
        N1_PCT = 0.0;
        N1Cmd = 0.0;
        N1rate = 15.0;
        fuelFlow = 0.0;
        Torque_PSI = 0.0;
        engTorque_ftlb = 0.0;
        availablePowerHP = 0.0; // PWR-01 reset the budget together with torque
        oilTemp = 15.0;
        oilPressure = 0.0;
        TOTcmd = 15.0;
        turbineOutletTemp = 15;
    }
    void initHot()
    {
        TOTrate = 10.0; addTemp = TOTwhenIgnited = 0.0; // RESET-03: no prior ignition history.
        engineState = ENG_Running;
        TargetN2 = 98.0;
        governorTrimN1 = 0.0;
        throttleInput = 1;
        starterButtonOn = false;
        N1_PCT = 75.0;
        N1Cmd = 75.0;
        N1rate = 15.0;
        fuelFlow = 0.0;
        engTorque_ftlb = 1400.0 * powerShare_;
        availablePowerHP = engTorque_ftlb * OmegaT / 550.0;
        Torque_PSI = 59.0 * getTorqueRelative(); // GAUGE-01: same normalization before first tick.
        oilTemp = 50.0;
        oilPressure = 85.0;
        TOTcmd = 600.0;
        turbineOutletTemp = 600;
    }
    /*
    void SetCommand(int command, const float value)
    {
        if (command == throttle)
        {
            throttleInput = value;
        }
        else if (command == starterButton)
        {
            starterButtonOn = value > 0.0;
        }
    }*/

    double getN1RPM()
    {
        return N1_PCT / 100.0;
    }
    double getTOT()
    {
        return turbineOutletTemp;
    }
    double getOilPress_Pa()
    {
        return oilPressure * Convert::PsiToPascal;
    }
    double getTorque_Nm()
    {
        return engTorque_ftlb * Convert::lbft_to_Nm;
    }
    double getTorqueRelative()
    {
        // API-04 / GAUGE-01: torque / reference torque, not horsepower / 425.
        // 100% is this model's aggregate 2,800 shp at nominal rotor speed;
        // not a validated CH-46 transmission or time-limited torque gauge.
        return engTorque_ftlb / (CH46::aggregatePowerHP * powerShare_ * 550.0 / OmegaT);
    }
    bool isStarterEngaged() const {
        // DBG-12: existing starter logic, without inventing a torque model.
        return starterButtonOn && (engineState==ENG_Motor || engineState==ENG_Ignition);
    }
    int stateForDebug() const { return static_cast<int>(engineState); }
    // DBG-13: expose the existing governor target without changing its law.
    double getGovernorTargetN2Pct() const {
        return engineState==ENG_Running ? TargetN2+
            LinInterp(p_flightControl.CollectiveInput,0.0,0.4,0.0,2.0) : 0.0;
    }
    double getFuelFlow()// [kg/hr]
    {
        return fuelFlow * Convert::lb_to_kg * powerShare_; // ENG-01: preserve aggregate consumption with both engines running.
    }
    double getAvailablePowerHP() const { return availablePowerHP; }
    double getEngTorque()
    {
        return engTorque_ftlb;
    }

    void update(const double frameTime, const bool hasFuel, const double currentN2)
    {
        if ((!hasFuel || throttleInput < -0.2) && engineState!= ENG_Motor)
        {//add flameout from thin air and damaged engine
            engineState = ENG_Off;
        }

        switch (engineState)
        {
        case ENG_Off:
        {
            governorTrimN1 = 0.0;
            if (starterButtonOn && N1_PCT < ENG_RPM_SUSTAIN_PCT)//TODO add req. electric power
            {
                engineState = ENG_Motor;
            }
            N1Cmd = 0.0;
            N1rate = limit(0.025 * pow(N1_PCT, 1.5), 0.75, 12);
            TOTcmd = p_EFMdata.ambientTemp_C;
            TOTrate = LinInterp(N1_PCT, 60.0, 5.0, 25.0, 1.5);
            fuelFlow = 0.0;

            break;
        }
        case ENG_Motor: //engine turning due to electrical starter. No fuel or ignition
        {
            governorTrimN1 = 0.0;
            if (starterButtonOn)
            {
                N1Cmd = ENG_RPM_MOTOR_PCT;
                double engineAccelFactor = 1.0 - limit(pow(N1_PCT / N1Cmd, 2.0), 0.0, 0.9);
                N1rate = 9.0 * engineAccelFactor;
            }
            else
            {
                engineState = ENG_Off;
            }

            if (throttleInput >= 0.0 && (hasFuel)) 
            {
                //igniterEngaged = true;
                engineState = ENG_Ignition;
                TOTwhenIgnited = turbineOutletTemp;
                addTemp = LinInterp(N1_PCT, 5.0, 11.0, 250.0, 0.0);// additional TOT rise due to early light off (creating a hot start if RPM too low)
            }
            TOTcmd = p_EFMdata.ambientTemp_C;
            TOTrate = LinInterp(N1_PCT, 5.0, 60.0, 1.5, 25.0);
            fuelFlow = 0.0;

            break;
        }
        case ENG_Ignition://20-40 sec from 19 to 58
        {
            governorTrimN1 = 0.0;
            if (starterButtonOn && hasFuel)
            {
                N1Cmd = ENG_RPM_IDLE_PCT;
                N1rate = 2.0;
            }
            else
            {
                engineState = ENG_Off;
            }

            if (N1_PCT >= ENG_RPM_SUSTAIN_PCT)
            {
                engineState = ENG_Running;
            }

            fuelFlow = 65.0 * pow((N1_PCT - ENG_RPM_MOTOR_MIN_PCT) / (ENG_RPM_SUSTAIN_PCT - ENG_RPM_MOTOR_MIN_PCT), 2.0);

            if (N1_PCT < 35.0) // TOT peaks around 35% then starts to go down
            {
                TOTcmd = 690 + TOTwhenIgnited + addTemp;
                TOTrate = LinInterp(N1_PCT, ENG_RPM_MOTOR_MIN_PCT, 35, 100.0, 50.0);
            }
            else
            {
                TOTcmd = LinInterp(N1_PCT, 35, ENG_RPM_IDLE_PCT, 690.0, 500.0) + TOTwhenIgnited + addTemp;
                TOTrate = LinInterp(N1_PCT, 35, N1Cmd, 50.0, 75.0);
            }

            break;
        }       
        case ENG_Running:
        {
            //---------- Engine governor ------------------------------------------------------------
            // modulates engine speed based on torque needed to keep rotor speed constant

            double N1Target = LinInterp(p_flightControl.CollectiveInput, 0.0, 1.0, 75.0, 102.0); ;// not the best way for base N1 target, but works for now

            //TargetN2 += 0.02 * N2Adjustment;
            //TargetN2 = limit(TargetN2, 96.0, 103.0);

            //An increase in collective from idle collective causes 1-1.5% increase N2
            double TargetN2Final = TargetN2 + LinInterp(p_flightControl.CollectiveInput, 0.0, 0.4, 0.0, 2.0);
            double N2error = TargetN2Final - currentN2;
            const double minN1 = ENG_RPM_IDLE_PCT;
            const double maxN1 = ENG_RPM_IDLE_PCT + throttleInput * (ENG_RPM_MAX_LIMIT_PCT - ENG_RPM_IDLE_PCT);
            const double proportionalCmd = N1Target + 3.0 * N2error;

            // GOV-01: proportional control alone required a permanent N2 error
            // (about +4.3% in the 0.1.12A hover trace). A slow, bounded trim
            // recentres the governor without changing its immediate response.
            if (std::isfinite(frameTime) && frameTime > 0.0 && std::isfinite(N2error))
            {
                const double candidateTrim = limit(governorTrimN1 + 0.35 * N2error * frameTime, -15.0, 15.0);
                const double candidateCmd = proportionalCmd + candidateTrim;
                if ((candidateCmd >= minN1 && candidateCmd <= maxN1) ||
                    (candidateCmd > maxN1 && N2error < 0.0) ||
                    (candidateCmd < minN1 && N2error > 0.0))
                {
                    governorTrimN1 = candidateTrim;
                }
            }

            N1Cmd = limit(proportionalCmd + governorTrimN1, minN1, maxN1);

            N1rate = limit(std::abs(N1_PCT - N1Cmd) * 1.5, 0.25, 15);//non-linear change rate
             //-----------------------------------------------------------------------------------

            // the manual only lists 78lb/h at 65% and 138lb/hr at 79%. max FF is 300lb/hr
            fuelFlow = LinInterp(N1_PCT, ENG_RPM_IDLE_PCT, ENG_RPM_MAX_LIMIT_PCT, 78.0, 300.0);

            updateTOT();
            break;
        }
        default:
            break;
        }


        N1Cmd = limit(N1Cmd, 0.0, ENG_RPM_MAX_LIMIT_PCT);
        N1_PCT = deltaLimit(N1_PCT, N1Cmd, N1rate, frameTime);

        double EnginePower_HP = powerShare_ * limit(fn_engHP.interpnf1(N1_PCT), 0.0, CH46::aggregatePowerHP);
        if (engineState != ENG_Running) EnginePower_HP = 0.0;
        //EnginePower_HP -= engAntiIceO-n ? 15.0 : 0.0;//15 SHP loss from anti ice on
        // PWR-01: use actual turbine speed with a finite nominal torque cap.
        // Aero repeats the budget calculation each rotor substep so a speed
        // rise within a host frame cannot keep the old excessive torque.
        availablePowerHP = EnginePower_HP;
        engTorque_ftlb = CH46::powerLimitedTorque(availablePowerHP,
            OmegaT*currentN2/100.0,OmegaT);
        
        //conversion for torque[lb-ft] to pressure[psi] based on chapter 7 in manual, says 500shp=70psi & 575=80.9psi. 500hp=5529ftlb; 575hp=6358ftlb <- what page??
        // GAUGE-01: preserve the inherited dial's 0..59 scale, but drive it
        // from model-relative torque. These are compatibility dial units,
        // not sourced CH-46 torque-pressure units. The old 425-hp divisor
        // made full CH-46 power display 6.59 times full scale and corrupted
        // the inherited temperature lookup. Temperature calibration is still provisional.
        Torque_PSI = 59.0 * getTorqueRelative();
        if (publishCockpit_) cockpitAPI.setParamNumber(ENGINE_TRQ, Torque_PSI);


        if (publishCockpit_) cockpitAPI.setCockpitDrawArg(INT_N1rpmNeedle, (float)(N1_PCT / 100));
        double intPart;
        double fractPart = modf(N1_PCT / 10.0, &intPart);
        if (publishCockpit_) cockpitAPI.setCockpitDrawArg(INT_N1rpmNeedleSmall, (float)fractPart);

        updateEngineOil(frameTime);
        turbineOutletTemp = deltaLimit(turbineOutletTemp, TOTcmd, TOTrate, frameTime);
        if (publishCockpit_) cockpitAPI.setParamNumber(ENGINE_TOT, turbineOutletTemp);

        if (publishCockpit_) G_Params.cautionLight[CL_EngOut] = engineState == ENG_Off;
    }

    void updateTOT()
    {
        // TODO added heat with scav air on
        // 10deg increase when anti-ice is on
        // TOT data from from fig.7-33 (pg.7-19)
        double startTmp = LinInterp(p_EFMdata.altitudeMSL_FT, 0, 10000, 435, 460);
        double endTmp = LinInterp(p_EFMdata.altitudeMSL_FT, 0, 10000, 705, 835);
        double baseTemp = LinInterp(Torque_PSI, 0, 70, startTmp, endTmp);// ambient temp 15C
        double ambientBias = (p_EFMdata.ambientTemp_C - 15) * 2.5;// based on chart, 10deg change in ambient~=25deg change in TOT
        TOTcmd = baseTemp + ambientBias;
        if (TOTcmd < turbineOutletTemp)
        {
            TOTrate = 25.0;
        }
        else
        {
            TOTrate = 40.0;
        }
    }

    void updateEngineOil(double frameTime)
    {
        //------- Oil temp ----------------------
        double engOilTempCmd_degC = 0.0;
        if (N1_PCT >= 20)
        {
            engOilTempCmd_degC = LinInterp(N1_PCT, 20.0, ENG_RPM_MAX_LIMIT_PCT, 5.0, 80.0);
        }
        engOilTempCmd_degC += p_EFMdata.ambientTemp_C;
        oilTemp = deltaLimit(oilTemp, engOilTempCmd_degC, 0.3, frameTime);      
        if (publishCockpit_) cockpitAPI.setCockpitDrawArg(INT_OilTempNeedle, (float)LinInterp(oilTemp, 15.0, 125.0, 0.0, 1.0));

        //------ Oil pressure ----------------------
        double engOilPress_tempBias = LinInterp(oilTemp, 5, 50, 1.5, 1.0);//make oil press higher when oil temp is cold

        double engOilPressCmd_psi = 0.0;
        //Engine Oil Pressure as a function of engine RPM 
        if (N1_PCT >= 94.0)
        {
            engOilPressCmd_psi = LinInterp(N1_PCT, 94.0, ENG_RPM_MAX_LIMIT_PCT, 115.0, 130.0);
        }
        else if (N1_PCT >= 79.0)
        {
            engOilPressCmd_psi = LinInterp(N1_PCT, 79.0, 94.0, 90.0, 115.0);
        }
        else
        {
            engOilPressCmd_psi = LinInterp(N1_PCT, 13.0, 79.0, 0.0, 90.0);
        }

        engOilPressCmd_psi *= engOilPress_tempBias;
        oilPressure = deltaLimit(oilPressure, engOilPressCmd_psi, 5.0, frameTime);
        if (publishCockpit_) cockpitAPI.setCockpitDrawArg(INT_OilPressNeedle, (float)(oilPressure / 200.0));
        if (publishCockpit_) G_Params.cautionLight[CL_XmsnPress] = oilPressure < 15;
    }
};
