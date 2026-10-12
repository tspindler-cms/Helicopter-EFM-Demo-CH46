#include "Aero.h"
#include "RotorKinematics.h"

//!!!!!!!!!!! TODO bugs/missing stuff: !!!!!!!!!!!!
//rotor wash factors

/// Helicopter flight model using blade element analysis
/// Source Data: 
/// 1) NASA TM-78629 (RSRA) -for math model equations
/// 2) NASA TM-88360 for clutch model
/// 3) NASA CR-166309 (GenHel) -similar to RSRA with small variations
/// 4) Helicopter Flight Dynamics by Gareth Padfield 3rd Edition -for generalized fuselage data
/// 5) Aerodynamic Tests of an Operational OH-6A Helicopter in the Ames Wind Tunnel -for specific Convert to OH-6
/// 6) NASA CR-3144 for specific constants to OH-6/AH-6
/// 
/*
Force and velocity units are imperial, angle units are in both radians and degrees.
All variables should be labeled with units denoted by brackets [unit].

Shaft axis is body axis with rotor shaft tilt (typically ~3deg forward).
Blade span axis is the local axis to each blade and
includes tangential, radial, and perpendicular components.
*/

CH46DAero::CH46DAero(EFMData& ptr_EFMdata, AH6JDamage& ptr_Damage, FlightControls& ptr_fltCntrl, bool aftRotor)
	: p_EFMdata(ptr_EFMdata)
	, p_Damage(ptr_Damage)
	, p_flightControl(ptr_fltCntrl)
	, isAft_(aftRotor)
{
	iS = isAft_ ? CH46::aftShaftTilt : CH46::frontShaftTilt;
	Initialize();
	if (!isAft_) aft_.reset(new CH46DAero(ptr_EFMdata, ptr_Damage, ptr_fltCntrl, true));
}


void CH46DAero::Initialize()
{
	//blade segment measurements
	XI = e / RMR;//hinge offset normalized to unit radius
	XIPrime = ePrime / RMR;//spar length, unit radius
	Y2[0] = sqrt(((1.0 - pow(XI + XIPrime, 2)) / (2.0 * NUM_BLADE_SEGMENTS)) + pow(XI + XIPrime, 2)) - XI;//1st segment distance
	double YINB[NUM_BLADE_SEGMENTS] = { 0.0 };//distance to inboard end of segment from centerline, unit radius
	double YOUTB[NUM_BLADE_SEGMENTS] = { 0.0 };//distance to outboard end of segment from centerline, unit radius
	YINB[0] = sqrt(pow(XI + Y2[0], 2) - (1.0 - pow(XI + XIPrime, 2)) / (2.0 * NUM_BLADE_SEGMENTS));
	YOUTB[0] = sqrt(pow(XI + Y2[0], 2) + (1.0 - pow(XI + XIPrime, 2)) / (2.0 * NUM_BLADE_SEGMENTS));
	DeltaY[0] = YOUTB[0] - YINB[0];
	
	for (int s = 1; s < NUM_BLADE_SEGMENTS; s++)
	{
		Y2[s] = sqrt(((1.0 - pow(XI + XIPrime, 2)) / NUM_BLADE_SEGMENTS) + pow(XI + Y2[s - 1], 2)) - XI;//2-nS segment distance
		YINB[s] = sqrt(pow(XI + Y2[s], 2) - (1.0 - pow(XI + XIPrime, 2)) / (2.0 * NUM_BLADE_SEGMENTS));
		YOUTB[s] = sqrt(pow(XI + Y2[s], 2) + (1.0 - pow(XI + XIPrime, 2)) / (2.0 * NUM_BLADE_SEGMENTS));
		DeltaY[s] = YOUTB[s] - YINB[s];
	}
	//cy mean chord of segment 

	TRSolidity = bNTR * cTR / (M_PI * RTR);
}
	
void CH46DAero::ResetRotorStates()
{
    frontLoadMean=aftLoadMean=shaftBalanceErrorNm=0;
    rotorSubsteps=1;rotorStepSeconds=0;
    appliedEnginePowerHP=appliedEngineTorque=clutchTorqueMean=0;
    clutchEngagedFraction=rotorSpeedLimitFraction=0;
    rollRateFeedback = yawRateFeedback = mixedRollInput = mixedPedalInput = 0.0;
    pitchAngleRate = pitchRateFeedback = mixedPitchInput = 0.0; // FCS-02 complete reset
    aeroForces.clear(); aeroMoments.clear();
    Omega = OmegaE = OmegaDot = OmegaTR = QMR = 0.0;
    PsiMR = 0.0; LambdaMR = CTA = DWMR = 0.0;
    LMR = MMR = NMR = XMR = YMR = ZMR = rotorThrustN_ = 0.0;
    skewAngleMR = 0.0; isClutchEngaged = false; isRotorBrakeEngaged = false;
    memset(Psi, 0, sizeof(Psi));
    memset(SinPsi, 0, sizeof(SinPsi));
    memset(CosPsi, 0, sizeof(CosPsi));
    memset(SinBeta, 0, sizeof(SinBeta));
    memset(CosBeta, 0, sizeof(CosBeta));
    memset(SinDelt, 0, sizeof(SinDelt));
    memset(CosDelt, 0, sizeof(CosDelt));
    memset(BetaDotDot, 0, sizeof(BetaDotDot));
    memset(BetaDot, 0, sizeof(BetaDot));
    memset(Beta, 0, sizeof(Beta));
    memset(DeltDotDot, 0, sizeof(DeltDotDot));
    memset(DeltDot, 0, sizeof(DeltDot));
    memset(Delt, 0, sizeof(Delt));
    memset(UP, 0, sizeof(UP));
    memset(UT, 0, sizeof(UT));
    memset(UR, 0, sizeof(UR));
    memset(Theta, 0, sizeof(Theta));
    memset(alphaTrans, 0, sizeof(alphaTrans));
    memset(CLY, 0, sizeof(CLY));
    memset(CDY, 0, sizeof(CDY));
    memset(FP, 0, sizeof(FP));
    memset(FT, 0, sizeof(FT));
    memset(FR, 0, sizeof(FR));
    memset(FXA, 0, sizeof(FXA));
    memset(FYA, 0, sizeof(FYA));
    memset(FZA, 0, sizeof(FZA));
    memset(MFA, 0, sizeof(MFA));
    memset(MLA, 0, sizeof(MLA));
    memset(MFD, 0, sizeof(MFD));
    memset(MLD, 0, sizeof(MLD));
    memset(FXT, 0, sizeof(FXT));
    memset(FYT, 0, sizeof(FYT));
    memset(FZT, 0, sizeof(FZT));
}

void CH46DAero::InitializeOff()
{
	ResetRotorStates();
	Omega = 0.0;
	OmegaE = 0.0;
	LambdaMR = 0.0;
	CTA = 0.01;
	DWMR = 0.01;
	if (aft_) aft_->InitializeOff();
}

void CH46DAero::InitializeOn()
{
	ResetRotorStates();
	Omega = OmegaT;
	OmegaE = OmegaT;
    OmegaTR = -OmegaT; // API-03: aft speed is initialized with the front, before the first tick.
	LambdaMR = -0.022;
	CTA = 0.014;
	DWMR = 0.022;
	if (aft_) aft_->InitializeOn();
}

void CH46DAero::update(double engtorque, double availablePowerHP)
{
    frontLoadMean=aftLoadMean=shaftBalanceErrorNm=0;
    appliedEnginePowerHP=appliedEngineTorque=clutchTorqueMean=0;
    clutchEngagedFraction=rotorSpeedLimitFraction=0;
	aeroForces.clear();
	aeroMoments.clear();
	if (p_EFMdata.deltaTime <= 0.0 || p_EFMdata.rho_SlgFt3 <= 0.0 || p_EFMdata.mass_kg <= 0.0) return;
    // NUM-01 (0.1.7 TEST): integrate fast rotor states at <=0.75 ms while the
    // host body state is held for its frame. Return the time-averaged forces
    // at their original positions, preserving every moment arm exactly.
    const double hostDt=p_EFMdata.deltaTime;
    const int steps=std::max(1,static_cast<int>(std::ceil(hostDt/0.00075-1e-12)));
    rotorSubsteps=steps;rotorStepSeconds=hostDt/steps;
    std::vector<ForceComponent> averagedForces;
    std::vector<Vec3> averagedMoments;
    averagedForces.reserve(7*steps);averagedMoments.reserve(3*steps);
    const double weight=1.0/steps;
    double frontThrustMean=0,aftThrustMean=0;
    double frontTorqueMean=0,aftTorqueMean=0,balanceErrorMean=0;
    double enginePowerMean=0,engineTorqueMean=0,clutchMean=0,engagedMean=0,speedLimitMean=0;
    p_EFMdata.deltaTime=hostDt/steps;
    for(int step=0;step<steps;++step) {
        evaluateRotors();
        FuselageModule();
        RotorDegreeOfFreedom(engtorque,availablePowerHP);
        frontTorqueMean+=QMR*weight;
        aftTorqueMean+=(aft_?aft_->QMR:0.0)*weight;
        balanceErrorMean+=shaftBalanceErrorNm*weight;
        enginePowerMean+=appliedEnginePowerHP*weight;
        engineTorqueMean+=appliedEngineTorque*weight;
        clutchMean+=clutchTorqueMean*weight;
        engagedMean+=clutchEngagedFraction*weight;
        speedLimitMean+=rotorSpeedLimitFraction*weight;
        frontThrustMean+=rotorThrustN_*weight;
        if(aft_)aftThrustMean+=aft_->rotorThrustN_*weight;
        for(auto force:aeroForces) {
            force.dir.x*=weight;force.dir.y*=weight;force.dir.z*=weight;
            averagedForces.push_back(force);
        }
        for(auto moment:aeroMoments) {
            moment.x*=weight;moment.y*=weight;moment.z*=weight;
            averagedMoments.push_back(moment);
        }
    }
    appliedEnginePowerHP=enginePowerMean;appliedEngineTorque=engineTorqueMean;
    frontLoadMean=frontTorqueMean; aftLoadMean=aftTorqueMean;
    shaftBalanceErrorNm=balanceErrorMean;
    clutchTorqueMean=clutchMean;clutchEngagedFraction=engagedMean;
    rotorSpeedLimitFraction=speedLimitMean;
    // Signed axial thrust observations use the same averaging interval.
    // Blade/inflow snapshots and shaft speed remain end-of-frame states.
    rotorThrustN_=frontThrustMean;
    if(aft_)aft_->rotorThrustN_=aftThrustMean;
    p_EFMdata.deltaTime=hostDt;
    aeroForces.swap(averagedForces);aeroMoments.swap(averagedMoments);
}

void CH46DAero::evaluateRotors()
{
	aeroForces.clear(); aeroMoments.clear();
	if (!aft_ || p_EFMdata.deltaTime <= 0.0 || p_EFMdata.rho_SlgFt3 <= 0.0) return;
    // 0.1.2: act through the existing tandem mixer. Keep raw pilot commands
    // intact for logging. Here r is aerodynamic yaw, hence DCS yaw rate is -r.
    // FCS-02: damp horizon-relative nose motion. Body q alone can be
    // nose-up while bank/yaw makes the nose fall; that rejected trial worsened
    // the recorded turn. No attitude target or direct force is introduced.
    pitchAngleRate=CH46::pitchAngleRate(q,-r,PhiB);
    const auto feedback=CH46::rateFeedback(p, -r, Omega/OmegaT, pitchAngleRate);
    pitchRateFeedback=feedback.pitch;
    const double pilotPitch=CH46::precisionAxis(p_flightControl.pitchOutput);
    const double pilotRoll=CH46::precisionAxis(p_flightControl.rollOutput);
    const double pilotPedal=CH46::precisionAxis(p_flightControl.PedalInput);
    mixedPitchInput=CH46::clamp(pilotPitch+feedback.pitch,-1.0,1.0);
    rollRateFeedback=feedback.roll; yawRateFeedback=feedback.pedal;
    mixedRollInput=CH46::clamp(pilotRoll+feedback.roll,-1.0,1.0);
    mixedPedalInput=CH46::clamp(pilotPedal+feedback.pedal,-1.0,1.0);
    const auto controls = CH46::mix(p_flightControl.CollectiveInput,
        mixedPitchInput, mixedRollInput, mixedPedalInput);
	MainRotorModule(controls.front.collective, controls.front.lateral, controls.front.longitudinal);
	aft_->aeroForces.clear(); aft_->aeroMoments.clear();
	// Both rotors share mechanical speed; inflow and individual blade states do not.
	aft_->Omega = Omega; aft_->OmegaDot = OmegaDot;
	aft_->MainRotorModule(controls.aft.collective, -controls.aft.lateral, controls.aft.longitudinal);
	// Undo reflection for forces (polar), positions (polar), and moments (axial).
	for (auto force : aft_->aeroForces) {
		force.dir.z = -force.dir.z; force.pos.z = -force.pos.z;
		aeroForces.push_back(force);
	}
	for (auto moment : aft_->aeroMoments) {
		moment.x = -moment.x; moment.y = -moment.y;
		aeroMoments.push_back(moment);
	}
	aft_->aeroForces.clear(); aft_->aeroMoments.clear();
}



// TODO:  
// check control input pitch amounts
void CH46DAero::MainRotorModule(double ThetaCUFF, double A1S, double B1S)
{
	const double hubX = isAft_ ? CH46::aftHubX : CH46::frontHubX;
	const double hubY = isAft_ ? CH46::aftHubY : CH46::frontHubY;
	const double mirror = isAft_ ? -1.0 : 1.0;
	// Velocities use lever arms from current CG; DCS force positions use model origin.
	const double lMR = (hubX - p_EFMdata.centerOfGravity.x) * Convert::meterToFoot;
	const double bMR = -mirror * p_EFMdata.centerOfGravity.z * Convert::meterToFoot;
	const double hMR = -(hubY - p_EFMdata.centerOfGravity.y) * Convert::meterToFoot;
	if (Omega < 0.01) { QMR = 0.0; rotorThrustN_ = 0.0; return; }

	// KIN-01 (0.1.5): callback acceleration already includes the rotation of
	// the body frame. The old extra omega x airspeed term counted it twice.
	// Retain hub-offset angular terms and subtract body-projected gravity.
	const Vec3 gravityBody(-Convert::gravity_fts2 * sin(ThetaB),
	    Convert::gravity_fts2 * sin(PhiB) * cos(ThetaB),
	    Convert::gravity_fts2 * cos(PhiB) * cos(ThetaB));
	const Vec3 hubAcceleration = CH46::hubSpecificAcceleration(
	    Vec3(VXBDot, VYBDot, VZBDot), gravityBody,
	    Vec3(p, q, r), Vec3(pDot, qDot, rDot), Vec3(lMR, bMR, hMR));
	const double VXHDot = hubAcceleration.x;
	const double VYHDot = hubAcceleration.y;
	const double VZHDot = hubAcceleration.z;

	// body translational velocities at the rotor hub
	double MuXH = (VXB + q * hMR - r * bMR) / (OmegaT * RMR);//x body axis velocity at MR hub, non dimensionalized
	double MuYH = (VYB - p * hMR + r * lMR) / (OmegaT * RMR);//y body axis velocity at MR hub, non dimensionalized
	double MuZH = (VZB - q * lMR + p * bMR) / (OmegaT * RMR);//z body axis velocity at MR hub, non dimensionalized

	//body axis to shaft axis transformation
	double VXSDot = VXHDot * cos(iS) - VZHDot * sin(iS);	//MR x shaft axis acceleration, [ft/sec^2]
	double VYSDot = VYHDot;									//MR y shaft axis acceleration, [ft/sec^2]
	double VZSDot = VXHDot * sin(iS) + VZHDot * cos(iS);	//MR z shaft axis acceleration, [ft/sec^2]
	double pDotS = pDot * cos(iS) - rDot * sin(iS);			//shaft axis roll acceleration, [rad/sec^2]
	double qDotS = qDot;									//shaft axis pitch acceleration, [rad/sec^2]
	double rDotS = pDot * sin(iS) + rDot * cos(iS);			//shaft axis yaw acceleration, [rad/sec^2]
	double MuXS = MuXH * cos(iS) - MuZH * sin(iS);			//x shaft axis velocity at MR hub
	double MuYS = MuYH;										//y shaft axis velocity at MR hub
	double MuZS = MuXH * sin(iS) + MuZH * cos(iS);			//z shaft axis velocity at MR hub
	double pS = p * cos(iS) - r * sin(iS);					//shaft axis roll rate, [rad/sec]
	double qS = q;											//shaft axis pitch rate, [rad/sec]
	double rS = p * sin(iS) + r * cos(iS);					//shaft axis yaw rate, [rad/sec]

	double MuTOT = sqrt(MuXS * MuXS + MuYS * MuYS + LambdaMR * LambdaMR);// total velocity at hub wth downwash, non dimensionalized
	MuTOT = limit(MuTOT, 0.001, 100.0); // prevent divide by zero	
	double KGL = sqrt(pow(MuXS, 2) + pow(MuYS, 2)) / MuTOT;//1st harmonic inflow coef
	double K1X = KGL * (MuXS / MuTOT);//cos component of 1st harmonic 
	double K1Y = KGL * (MuYS / MuTOT);//sin component of 1st harmonic 

	// Downwash and inflow calculation
	double Klambda = KlambdaPrime / p_EFMdata.deltaTime;//downwash filter constant
	DWMR = ((Klambda - 1.0) / Klambda) * DWMR + (1.0 / Klambda) * (CTA / MuTOT);// downwash normalized (ie 1==OmegaT * RMR)
	LambdaMR = MuZS - DWMR;//inflow

	// Main Rotor rotational position
	//Omega = OmegaT;// uncomment to disable rotor degree of freedom (ie make rotor speed constant)
	double DeltaPsi = Omega * p_EFMdata.deltaTime;//MR advance angle, [rad]
	PsiMR += DeltaPsi;//MR rotational position
	if (PsiMR > M_PI)
	{
		PsiMR -= 2.0 * M_PI;
	}

	// Individual blade rotations
	// zero azimuth is aligned over tail (i.e. negative X axis)
	Psi[0] = PsiMR;
	for (int b = 1; b < NUM_BLADES; b++)
	{
		Psi[b] = Psi[b - 1] + 2.0 * M_PI / NUM_BLADES;
		if (Psi[b] > M_PI)
		{
			Psi[b] -= 2.0 * M_PI;
		}
	}

    // GEO-02: use actual world-vertical hub offset for the current attitude.
    // The aft mirrored frame has the same pitch and an on-centre hub, so
    // cos(PhiB) gives the same physical height. Flat local ground remains
    // an approximation; no terrain sample at each hub is supplied by DCS.
    const double hubHeightM = CH46::verticalOffset(hubX,hubY,0.0,ThetaB,PhiB);
	double Z_rotor = limit(p_EFMdata.altitudeAGL_ft + hubHeightM * Convert::meterToFoot, 1.0, 1000.0);
	double denomGE = sqrt(pow(MuXS, 2) + pow(MuYS, 2) + pow(LambdaMR, 2));
	denomGE = (denomGE > 1e-6) ? denomGE : 1e-6; // avoid divide-by-zero
	double baseGE = 1.0 + 0.13 * pow(RMR / Z_rotor, 2) * LambdaMR / denomGE; // LambdaMR < 0 => base can go negative for large RMR / low Z_rotor
	baseGE = (baseGE > 1e-6) ? baseGE : 1e-6;   // pow(negative, -2/3) is NaN; clamp so Kge stays defined
	double Kge = limit(pow(baseGE, -2.0 / 3.0), 1.0, 1.5);//ground effect gain factor

	//---------------------------
	//----Calculations for each blade-------
	for (int b = 0; b < NUM_BLADES; b++)
	{
		// pre-calculate commonly used functions
		SinPsi[b] = sin(Psi[b]);
		CosPsi[b] = cos(Psi[b]);
		//-------- Blade flapping angle and rates----------------------
		double OmegaB = limit(Omega, OmegaT / 10.0, OmegaT * 5.0);//this variable is only for beta and betaDot equations to avoid divide by 0
		Beta[b] = Beta[b] + BetaDot[b] * sin(DeltaPsi) / OmegaB + BetaDotDot[b] * (1.0 - cos(DeltaPsi)) / pow(OmegaB, 2);
		Beta[b] = limit(Beta[b], betaDown, betaUp);//limit to avoid divide by zero. flapping angle shouldn't get past ~30deg anyways
		

		BetaDot[b] = BetaDot[b] * cos(DeltaPsi) + BetaDotDot[b] * sin(DeltaPsi) / OmegaB;
		BetaDot[b] = limit(BetaDot[b], -50.0, 50.0);

		BetaDotDot[b] = (Mb / Ib) * (cos(Beta[b]) * (VZSDot + e * (2.0 * Omega * (pS * CosPsi[b] - qS * SinPsi[b]) + pDotS * SinPsi[b] + qDotS * CosPsi[b])) + sin(Beta[b]) * cos(Delt[b]) * (VYSDot * SinPsi[b] - VXSDot * CosPsi[b] - e * pow(Omega - rS, 2)) + sin(Delt[b]) * sin(Beta[b]) * (VXSDot * SinPsi[b] + VYSDot * CosPsi[b] - e * (rDotS - OmegaDot)))
			+ pow(cos(Beta[b]), 2) * (cos(Delt[b]) * (pDotS * SinPsi[b] + qDotS * CosPsi[b] - 2.0 * (DeltDot[b] + Omega) * (qS * SinPsi[b] - pS * CosPsi[b])) - sin(Delt[b]) * (2.0 * (Omega + DeltDot[b]) * (pS * SinPsi[b] + qS * CosPsi[b]) + qDotS * SinPsi[b] - pDotS * CosPsi[b]))
			+ cos(Beta[b]) * sin(Beta[b]) * (2.0 * DeltDot[b] * (rS - Omega) - pow(Omega - rS, 2) - pow(DeltDot[b], 2))
			+ (MFA[b] / Ib) + (MFD[b] / Ib);
		BetaDotDot[b] = limit(BetaDotDot[b], -2000.0, 2000.0);

		// pre-calculate commonly used functions
		SinBeta[b] = sin(Beta[b]);
		CosBeta[b] = cos(Beta[b]);

#ifdef USE_LAG_DOF
		//---------- Blade lagging angle and rates----------------------------
		Delt[b] = Delt[b] + DeltDot[b] * sin(DeltaPsi) / OmegaB + DeltDotDot[b] * (1.0 - cos(DeltaPsi)) / pow(OmegaB, 2);
		Delt[b] = limit(Delt[b], deltAFT, deltFWD);

		DeltDot[b] = DeltDot[b] * cos(DeltaPsi) + DeltDotDot[b] * sin(DeltaPsi) / OmegaB;
		DeltDot[b] = limit(DeltDot[b], -500.0, 500.0);

		DeltDotDot[b] = (Mb / (Ib * cos(Beta[b]))) * (sin(Delt[b]) * (VYSDot * SinPsi[b] - VXSDot * CosPsi[b] - e * pow(Omega - rS, 2)) - cos(Delt[b]) * (VXSDot * SinPsi[b] + VYSDot * CosPsi[b] + e * (OmegaDot - rDotS)))
			+ (sin(Beta[b]) / cos(Beta[b])) * (2.0 * BetaDot[b] * (Omega + DeltDot[b] - rS) + qDotS * sin(Psi[b] + Delt[b]) - pDotS * cos(Psi[b] + Delt[b])) + (rDotS - OmegaDot)
			+ 2.0 * BetaDot[b] * (cos(Delt[b]) * (qS * SinPsi[b] - pS * CosPsi[b]) + sin(Delt[b]) * (pS * SinPsi[b] + qS * CosPsi[b]))
			- MLA[b] / (Ib * cos(Beta[b])) - MLD[b] / (Ib * cos(Beta[b]));
		DeltDotDot[b] = limit(DeltDotDot[b], -2000.0, 2000.0);		
#endif
		// pre-calculate commonly used functions
		SinDelt[b] = sin(Delt[b]);
		CosDelt[b] = cos(Delt[b]);

		//-----------------------------
		//----Calculations for each segment--------
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++) //calculate for each segment
		{
			//Blade segment velocities (blade span axis)
			UP[b][s] = LambdaMR * CosBeta[b] + MuYS * SinBeta[b] * sin(Psi[b] + Delt[b]) - MuXS * SinBeta[b] * cos(Psi[b] + Delt[b])
				+ XI * CosBeta[b] * ((qS / OmegaT - K1X * DWMR) * CosPsi[b] + (pS / OmegaT + K1Y * DWMR) * SinPsi[b])
				+ XI * SinBeta[b] * SinDelt[b] * ((Omega - rS) / OmegaT)
				+ Y2[s] * (-(BetaDot[b] / OmegaT) + (qS / OmegaT - K1X * DWMR * CosBeta[b]) * cos(Psi[b] + Delt[b]) + (pS / OmegaT + K1Y * DWMR * CosBeta[b]) * sin(Psi[b] + Delt[b]));

			UT[b][s] = MuXS * sin(Psi[b] + Delt[b]) + MuYS * cos(Psi[b] + Delt[b])
				+ XI * CosDelt[b] * ((Omega - rS) / OmegaT)
				+ Y2[s] * ((DeltDot[b] / OmegaT) + (pS / OmegaT * cos(Psi[b] + Delt[b]) - qS / OmegaT * sin(Psi[b] + Delt[b])) * SinBeta[b] + CosBeta[b] * ((Omega - rS) / OmegaT));
			if (UT[b][s] == 0.0) { UT[b][s] = 0.00001; }

			UR[b][s] = LambdaMR * SinBeta[b] + MuXS * CosBeta[b] * cos(Psi[b] + Delt[b]) - MuYS * CosBeta[b] * sin(Psi[b] + Delt[b])
				+ XI * SinBeta[b] * ((qS / OmegaT - K1X * DWMR) * CosPsi[b] + (pS / OmegaT + K1Y * DWMR) * SinPsi[b])
				- XI * CosBeta[b] * SinDelt[b] * ((Omega - rS) / OmegaT)
				+ Y2[s] * SinBeta[b] * (-K1X * DWMR * cos(Psi[b] + Delt[b]) + K1Y * DWMR * sin(Psi[b] + Delt[b]));

			// Resultant velocity at blade segment
			double UYAW = sqrt(pow(UT[b][s], 2) + pow(UP[b][s], 2) + pow(UR[b][s], 2));//blade segment resultant velocity, non dimen

			//blade segment flow skew angle, non dimen
			double cosGamma = std::abs(UT[b][s]) / sqrt(pow(UT[b][s], 2) + pow(UR[b][s], 2));
			if (cosGamma == 0.0) { cosGamma = 0.00001; }//to avoid divide by zero later

			// Blade segment geometric pitch angle
			Theta[b][s] = (ThetaCUFF - A1S * cos(Psi[b] + DeltaSP) - B1S * sin(Psi[b] + DeltaSP) + Theta1 * (Y2[s] - XIPrime) - 57.3 * Beta[b] * tan(delt3)) * Convert::degToRad;// + Kalpha1 * (57.3 * Delt[b]) + Kalpha2 * pow(57.3 * Delt[b], 2);//rad


			//ThetaA[b][n] + pow(ThetaA[b][s], 3) / 3.0 + (2.0 / 15.0) * pow(ThetaA[b][s], 5) = tan(ThetaA[b][s])
			//blade segment angle of attack, [deg]
			double alphaY = atan2((UT[b][s] * tan(Theta[b][s]) + UP[b][s]) * cosGamma, UT[b][s] - UP[b][s] * tan(Theta[b][s]) * pow(cosGamma, 2)) * Convert::radToDeg;
			alphaY = limit(alphaY, -180.0, 180.0);


			//yawed alpha is modified here for input to CL lookup table (sweep theory-> unyawed alpha)
			if (std::abs(alphaY) >= 0 && std::abs(alphaY) <= 13.5 / cosGamma)
			{
				if (std::abs(alphaY) > 90.0)
				{
					alphaTrans[b][s] = std::abs(alphaY * cosGamma + alphaY / std::abs(alphaY) * 180.0 * (1.0 - cosGamma));
				}
				else
				{
					alphaTrans[b][s] = std::abs(alphaY * cosGamma);
				}
			}
			else if (std::abs(alphaY) >= (180.0 - 8.0 / cosGamma) && std::abs(alphaY) <= 180.0)
			{
				if (std::abs(alphaY) > 90.0)
				{
					alphaTrans[b][s] = std::abs(alphaY * cosGamma + alphaY / std::abs(alphaY) * 180.0 * (1.0 - cosGamma));
				}
				else
				{
					alphaTrans[b][s] = std::abs(alphaY * cosGamma);
				}
			}
			else if (std::abs(alphaY) >= 13.5 / cosGamma && std::abs(alphaY) <= (180.0 - 8.0 / cosGamma))
			{
				alphaTrans[b][s] = std::abs(alphaY);
			}
			alphaTrans[b][s] = limit(alphaTrans[b][s], 0.0, 180.0);
			if (alphaY < 0.0)
			{
				//alphaTrans[b][s] = -alphaTrans[b][s];
			}

			//double MachS = pow(UT[b][s] * UT[b][s] + UP[b][s] * UP[b][s], 0.5) * OmegaT * RMR / p_EFMdata.speedOfSound_fts;// blade segment mach, for Cd&Cl table lookup

			// Blade segment lift and drag coefficients
			double CLfinal = 0.0;
			double CDfinal = 0.0;
			//if (alphaTrans[b][s] <= 30.0)
			//{
			//	CLfinal = _CL_NACA0015_30(alphaTrans[b][s], MachS);
			//	CDfinal = _CD_NACA0015_30(std::abs(alphaY), MachS);
			//}
			//else if (alphaTrans[b][s] > 30.0)
			//{
			CLfinal = fn_CL_NACA0015.interpnf1(alphaTrans[b][s]);// _CL_NACA0015(alphaTrans[b][s]);
			CDfinal = fn_CD_NACA0015.interpnf1(std::abs(alphaY));//_CD_NACA0015(std::abs(alphaY));
			//}
			
			if (s < NUM_BLADE_SEGMENTS - 1)
			{
				CLY[b][s] = CLfinal * 1.0;// _CL_NACA0015(alphaTrans[b][n]);
			}
			if (s == NUM_BLADE_SEGMENTS - 1)//last segment only
			{
				CLY[b][s] = (1.0 - ((1.0 - BMR) / DeltaY[NUM_BLADE_SEGMENTS - 1])) * CLfinal;// _CL_NACA0015(alphaTrans[b][n]);
			}
			if (alphaY < 0.0)
			{
				CLY[b][s] = -CLY[b][s];
			}

			CDY[b][s] = CDfinal;// _CD_NACA0015(limit(std::abs(alphaY), 0.0, 180.0)) + DeltaCD;

			// Blade segment forces (blade span axis)
			FP[b][s] = 0.5 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 3) * (CR * DeltaY[s]) * UYAW * (CLY[b][s] * (UT[b][s] / cosGamma) + CDY[b][s] * UP[b][s]);// main source of lift
			FT[b][s] = 0.5 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 3) * (CR * DeltaY[s]) * UYAW * (CDY[b][s] * UT[b][s] - CLY[b][s] * UP[b][s] * cosGamma);
			FR[b][s] = 0.5 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 3) * (CR * DeltaY[s]) * UYAW * (CDY[b][s] - CLY[b][s] * (UP[b][s] / UT[b][s]) * cosGamma) * UR[b][s];

		}// end of blade segment calculations

		//Blade aerodynamic shears (blade span axis)
		double FPB = 0.0;//blade shear force perp., [lb]
		double FTB = 0.0;//blade shear force tangential, [lb]
		double FRB = 0.0;//blade shear force radial, [lb]
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			FPB += FP[b][s];
			FTB += FT[b][s];
			FRB += FR[b][s];
		}


		// Blade aerodynamic shears (shaft axis)
		FXA[b] = FRB * CosBeta[b] * SinDelt[b] - FTB * CosDelt[b] - FPB * SinBeta[b] * SinDelt[b];
		FYA[b] = FRB * CosBeta[b] * CosDelt[b] + FTB * SinDelt[b] - FPB * SinBeta[b] * CosDelt[b];
		FZA[b] = -(FRB * SinBeta[b] + FPB * CosBeta[b]);

		// Moments about blade flap+lag hinges
		MFA[b] = 0.0;
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			MFA[b] += Y2[s] * FP[b][s];
		}
		MFA[b] = MFA[b] * RMR;

		MLA[b] = 0.0;
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			MLA[b] += Y2[s] * FT[b][s];
		}
		MLA[b] = MLA[b] * RMR;

		// Blade hinge contraint moments
		MFD[b] = -(Kbeta * Beta[b] + KbetaDot * BetaDot[b]);// this is 0 for articulated rotors, non-zero for hingeless rotors
		
#ifdef USE_LAG_DOF
		//double FdeltDot = fn_lagDamp.interpnf1(limit(std::abs(DeltDot[b]), 0.0, 0.82));// lagging hinge damper const, [ft-lb-sec/rad]
		//if (DeltDot[b] < 0.0)
		//{
		//	FdeltDot = -FdeltDot;
		//}
		//MLD[b] = 1450.0 * DeltDot[b];// LagDampArm * FdeltDot;
#endif

		// Total blade shear forces at hinge (shaft axis)
		// DCS integrates the whole aircraft including blade mass. Centrifugal
		// and blade-weight reactions are internal loads, not additional external
		// forces. Keep inertia in the flap ODE, export only aerodynamic loads.
		FXT[b] = FXA[b];
		FYT[b] = FYA[b];
		FZT[b] = FZA[b] * Kge;

		double bladeHealth = p_Damage.bladeHealth(isAft_,b);
		FXT[b] *= bladeHealth;
		FYT[b] *= bladeHealth;
		FZT[b] *= bladeHealth;

		// convert rotating shaft axis to rotor hub axis
		double XB = -(FYT[b] * CosPsi[b] - FXT[b] * SinPsi[b]);
		double YB = (FXT[b] * CosPsi[b] + FYT[b] * SinPsi[b]);
		double ZB = FZT[b];

		ForceComponent bladeForce;
		bladeForce.dir.x = (XB * cos(iS) + ZB * sin(iS)) * Convert::lbf_to_N;
		bladeForce.dir.y = -(-XB * sin(iS) + ZB * cos(iS)) * Convert::lbf_to_N;
		bladeForce.dir.z = YB * Convert::lbf_to_N;
        // GEO-01: the hinge offset is a SHAFT-frame position vector. Rotate
        // it through the same shaft tilt as the force above before adding
        // the body-frame hub location. Previously the hinge circle stayed
        // horizontal while the forces tilted, corrupting r cross F moments.
        // Shaft offset (-e*cos(psi), e*sin(psi), 0) becomes DCS x/up/right:
        // (-e*cos(psi)*cos(iS), -e*cos(psi)*sin(iS), e*sin(psi)).
        // The existing aft reflection is applied later to both force and
        // position. This corrects geometry; it does not tune control gains
        // or establish that the provisional shaft tilts/hinge offset are real.
        bladeForce.pos.x = hubX - e * CosPsi[b] * cos(iS) * Convert::feetToMeter;
        bladeForce.pos.y = hubY - e * CosPsi[b] * sin(iS) * Convert::feetToMeter;
        bladeForce.pos.z = e * SinPsi[b] * Convert::feetToMeter;
		aeroForces.push_back(bladeForce);



	}// end of individual blade calculations


	/*
	//double T = 0.0;//Total thrust force shaft axis, [lb]
	for (int b = 0; b < NUM_BLADES; b++)
	{
	//	T += FZT[b]* p_Damage.bladeHealth(isAft_,b);
	}
	//T *= KGE;

	//double H = 0.0;//Total drag force shaft axis, [lb]
	for (int b = 0; b < NUM_BLADES; b++)
	{
		//H += (FYT[b] * CosPsi[b] - FXT[b] * SinPsi[b])* p_Damage.bladeHealth(isAft_,b);
	}
	//H = -H;

	//double J = 0.0;//Total side force shaft axis, [lb]
	for (int b = 0; b < NUM_BLADES; b++)
	{
		//J += (FXT[b] * CosPsi[b] + FYT[b] * SinPsi[b])* p_Damage.bladeHealth(isAft_,b);
	}
	*/
	double TA = 0.0;//aerodynamic thrust for downwash calc, [lb]
	for (int b = 0; b < NUM_BLADES; b++)
	{
		TA += FZA[b] * p_Damage.bladeHealth(isAft_,b);
	}
	TA = -TA;
	rotorThrustN_ = TA * Convert::lbf_to_N;

	CTA = TA / (2.0 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 4) * M_PI);// MR thrust coef used for downwash calc


	double LH = 0.0;//MR rolling moment shaft axis, [ft-lb]
	//for (int b = 0; b < NUM_BLADES; b++)
	//{
		//LH += ((MFD[b] * CosDelt[b] - MLD[b] * SinBeta[b] * SinDelt[b]) * SinPsi[b] + (MFD[b] * SinDelt[b] + MLD[b] * SinBeta[b] * CosDelt[b]) * CosPsi[b])* p_Damage.bladeHealth(isAft_,b);
	//}

	double MH = 0.0;//MR pitching moment shaft axis, [ft-lb]
	//for (int b = 0; b < NUM_BLADES; b++)
	//{
		//MH += ((MFD[b] * CosDelt[b] - MLD[b] * SinBeta[b] * SinDelt[b]) * CosPsi[b] + (MFD[b] * SinDelt[b] + MLD[b] * SinBeta[b] * CosDelt[b]) * SinPsi[b])* p_Damage.bladeHealth(isAft_,b);
	//}

	double Q = 0.0;//MR torque shaft axis, [ft-lb]
	for (int b = 0; b < NUM_BLADES; b++)
	{
		//Q += (e* FXT[b] + MLD[b] * CosBeta[b]);//note: if lag DOF enabled
        // ROTOR-01: FXT already includes blade integrity, while MLA does not.
        // Scale each contribution once. The old expression multiplied the
        // hinge-offset shear term by integrity twice for partially damaged blades.
        Q += e * FXT[b] - MLA[b] * CosBeta[b] * p_Damage.bladeHealth(isAft_,b);
	}
	Q = -Q;
	QMR = Q;

	double Q2 = 0.0;//MR torque shaft axis, [ft-lb]
	for (int b = 0; b < NUM_BLADES; b++)
	{
		//Q += (e* FXT[b] + MLD[b] * CosBeta[b]);//note: if lag DOF enabled
		Q2 += (- MLA[b] * CosBeta[b])* p_Damage.bladeHealth(isAft_,b);// if no lag DOF
	}
	Q2 = -Q2; // full reaction torque; opposite rotor is explicitly counter-rotating

	// Transform the aerodynamic couple from shaft to body axes.
    // DYN-01: shear forces and their hinge moment arms are exported for this
    // substep. Export the remaining aerodynamic couple for the SAME instant.
    // Filtering only this couple by 0.15 s made the total shaft-axis moment
    // disagree with QMR during load changes, including roll from tilted shafts.
    // Aero::update already time-averages every force and moment over the host
    // tick. No extra drivetrain elasticity/lag state exists to store this load.
	LMR = LH * cos(iS) + Q2 * sin(iS);
	MMR = MH;
	NMR = -LH * sin(iS) + Q2 * cos(iS);



	double a1SF = 0.0;//fourier series coef, flapping, [deg]
	for (int b = 0; b < NUM_BLADES; b++)
	{
		a1SF += Beta[b] * CosPsi[b] * p_Damage.bladeHealth(isAft_,b);
	}
	a1SF *= -2.0 / NUM_BLADES * Convert::radToDeg;

	skewAngleMR = atan2(MuXS, std::abs(LambdaMR)) * Convert::radToDeg + a1SF;//[deg], angle between rotor wake and MR shaft (i.e. is rotor wake straight down or washed rearward due to forward velocity)

	double animPos = PsiMR / (2.0*M_PI);//animation is 0-1, but PsiMR is -Pi to Pi
	if (animPos < 0)
	{
		animPos += 1.0;
	}
	const double displayPhase = isAft_ ? fmod(1.0 - animPos + 1.0 / 6.0, 1.0) : animPos;
	cockpitAPI.setExternalDrawArg(isAft_ ? EXT_TRspin : EXT_RotorSpin, (float)displayPhase);
	cockpitAPI.setExternalDrawArg(EXT_RotorDroop, (float)LinInterp(Omega, 0, OmegaT * 0.1, -1.0f, 0.0f));
	

    // DYN-01: forces and their remaining couple are valid together, including
    // the first rotating step. Do not delay only the couple at initialization.
    Vec3 mrMoment;
    mrMoment.x = LMR * Convert::lbft_to_Nm;
    mrMoment.y = -NMR * Convert::lbft_to_Nm;
    mrMoment.z = MMR * Convert::lbft_to_Nm;
    aeroMoments.push_back(mrMoment);

}

//todo add rearward n coef table
//todo validate wash factor tables
void CH46DAero::FuselageModule()
{
	double EKFX = fn_EKFX.interpnf1(limit(skewAngleMR, -20.0, 90.0));//rotor wash factor on fuselage
	double EKFZ = fn_EKFZ.interpnf1(limit(skewAngleMR, -20.0, 90.0));//rotor wash factor on fuselage

	double VXF = VXB + EKFX * (DWMR * OmegaT * RMR);//fuselage x body axis velocity, [ft/s]
	double VYF = VYB;								//fuselage y body axis velocity, [ft/s]
	double VZF = VZB - EKFZ * (DWMR * OmegaT * RMR);//fuselage z body axis velocity, [ft/s]
	double qF = 0.5 * p_EFMdata.rho_SlgFt3 * (VXF * VXF + VYF * VYF + VZF * VZF);//dynamic pressure at fuselage, [lb/ft^2]

	double alphaF = atan2(VZF, VXF);//AoA of fuselage, [rad]
	alphaF_deg = alphaF * Convert::radToDeg;
	double speedF = sqrt(VXF * VXF + VYF * VYF + VZF * VZF);
	double BetaF = speedF > 1e-9 ? asin(limit(VYF / speedF, -1.0, 1.0)) : 0.0;
	double BetaF_deg = BetaF * Convert::radToDeg;
	PsiFdeg = -BetaF * Convert::radToDeg;

	double Xf = qF * SPF * fn_Cx_Fuselage.interpnf1(alphaF_deg);
	double Yf = qF * SSF * fn_Cz_Fuselage.interpnf1(BetaF_deg);
	double Zf = qF * SPF * fn_Cz_Fuselage.interpnf1(alphaF_deg);

	double Mf = qF * SPF * fn_Cm_Fuselage.interpnf1(alphaF_deg);
	double Nf = qF * SSF * fn_Cna_Fuselage.interpnf1(BetaF_deg);


	ForceComponent FuseForce;
	FuseForce.dir.x = Xf * Convert::lbf_to_N;
	FuseForce.dir.y = -Zf * Convert::lbf_to_N;
	FuseForce.dir.z = Yf * Convert::lbf_to_N;
	FuseForce.pos.x = 0;
	FuseForce.pos.y = 0;
	FuseForce.pos.z = 0;
	aeroForces.push_back(FuseForce);

	Vec3 fuseMoment;
	fuseMoment.x = 0;
	fuseMoment.y = -Nf * Convert::lbft_to_Nm;
	fuseMoment.z = Mf * Convert::lbft_to_Nm;
	aeroMoments.push_back(fuseMoment);

}//end fuselage module

// todo: add TR torque?, tune inertias/ratios
void CH46DAero::RotorDegreeOfFreedom(double engtorque, double availablePowerHP)
{
	/* Summary of operation
	* Change in collective causes change in MR torque which changes the speed of the rotor, if engine torque is constant.
	* Engine governor tries to keep constant N2 speed, therefore with a change in N2 speed,
	* the governor will change fuel flow, which changes N1, which changes N2 speed and engine torque output.
	* this change in engine torque will correct rotor speed then match required torque of MR
	* // OmegaDot (angular accel)=torque/inertia
	*/


	// torque "required/used" by main rotor	
	const double loadTorque = QMR + (aft_ ? aft_->QMR : 0.0);
	double rotorBrakeTorque = 0.0;
	if (isRotorBrakeEngaged)// additional torque from rotor brake to help MR stop sooner
	{
		rotorBrakeTorque = LinInterp(Omega, OmegaT * 0.4, 0.0, 200, 500);
	}
    // DYN-02: the shaft momentum balance must use the same aerodynamic load
    // that the blades produce now. The former 0.15 s low-pass load introduced
    // an unmodeled torque impulse during each load change. Aerodynamic load
    // smoothing has no physical storage element in this rigid-shaft model.
	double netMRTorque = -loadTorque - rotorBrakeTorque;

    // PWR-01: the engine map supplies a power budget; enforce it using the
    // actual free-turbine speed at every rotor substep. The negative sentinel
    // preserves explicit fixed-torque calls in existing standalone tests.
    const double QEng = availablePowerHP>=0.0 ?
        CH46::powerLimitedTorque(availablePowerHP,OmegaE,OmegaT) : std::max(0.0,engtorque);
    appliedEngineTorque=QEng;
    appliedEnginePowerHP=QEng*OmegaE/550.0;
    const double N2torque=LinInterp(OmegaE,1.0,OmegaT,25.0,600.0);
    const double netEngTorque=QEng-N2torque;
    const double oldOmega=Omega;
    const double oldEngineOmega=OmegaE;
    const auto shafts=CH46::advanceShafts(Omega,OmegaE,netMRTorque,
        netEngTorque,JMR,JE,p_EFMdata.deltaTime);
    isClutchEngaged=shafts.engaged;
    clutchTorqueMean=shafts.clutchTorque;
    clutchEngagedFraction=shafts.engaged?1.0:0.0;
    // DBG-11: retain the old numerical ceiling and explicitly report when
    // it intervenes. It is not a real overspeed governor or damage model.
    rotorSpeedLimitFraction=(shafts.rotorOmega>OmegaT*1.2)?1.0:0.0;
    Omega=limit(shafts.rotorOmega,0.0,OmegaT*1.2);
    OmegaE=limit(shafts.engineOmega,0.0,OmegaT*1.2);
    // DBG-13: the clutch impulse is internal to these two shafts and cancels.
    // This residual should be roundoff-sized away from numerical speed/zero
    // clamps. Report a clamp impulse rather than concealing it in the balance.
    shaftBalanceErrorNm=((JMR*(Omega-oldOmega)+JE*(OmegaE-oldEngineOmega))
        /p_EFMdata.deltaTime-netMRTorque-netEngTorque)*Convert::lbft_to_Nm;
    OmegaDot=(Omega-oldOmega)/p_EFMdata.deltaTime;
	OmegaTR = -Omega; // aft lifting rotor, not a high-speed antitorque rotor


	//----- Outputs for lua indicator script -----
	cockpitAPI.setParamNumber(EFM_N2_RPM, OmegaE / OmegaT * 100.0);
	cockpitAPI.setParamNumber(EFM_NR_RPM, Omega / OmegaT * 100.0);
}
