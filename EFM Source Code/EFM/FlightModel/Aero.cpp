#include "Aero.h"
#include <cmath>

// Set to 1 to log engine torque and flight controls before rotor integration (view with DebugView / MSVC output).
#ifndef CH46_LOG_PRE_ROTOR
#define CH46_LOG_PRE_ROTOR 0
#endif
#if CH46_LOG_PRE_ROTOR
#include <cstdio>
#endif

//!!!!!!!!!!! TODO bugs/missing stuff: !!!!!!!!!!!!
//rotor wash factors

/// Helicopter flight model using blade element analysis
/// Adapted for CH-46D Sea Knight tandem rotor (vwv_ch46d_base.lua reference)
/// Source Data: 
/// 1) NASA TM-78629 (RSRA) - math model equations
/// 2) NASA TM-88360 - clutch model
/// 3) NASA CR-166309 (GenHel) - similar to RSRA
/// 4) Helicopter Flight Dynamics by Gareth Padfield 3rd Ed. - generalized fuselage data
/// 5) vwv_ch46d_base.lua - CH-46D geometry, weights, rotor_MOI, centering, tandem efficiency
/// 
/*
Force and velocity units are imperial, angle units are in both radians and degrees.
All variables should be labeled with units denoted by brackets [unit].

Shaft axis is body axis with rotor shaft tilt (typically ~3deg forward).
Blade span axis is the local axis to each blade and
includes tangential, radial, and perpendicular components.
*/

CH46DAero::CH46DAero(EFMData& ptr_EFMdata, AH6JDamage& ptr_Damage, FlightControls& ptr_fltCntrl)
	: p_EFMdata(ptr_EFMdata)
	, p_Damage(ptr_Damage)
	, p_flightControl(ptr_fltCntrl)
{
	Initialize();
}


void CH46DAero::Initialize()
{
	rotorCfgFront_ = RotorDiskConfig{
		FSMR, WLMR, BLMR, iS, 1.0, 0.0, false, 1.0, 1.0, 0.0, static_cast<int>(EXT_RotorSpin), RotorSkewOutput::ReplaceFuselageSkew };
	rotorCfgRear_ = RotorDiskConfig{
		FSRR, WLRR, BLRR, iSRR, rearRotationSign, rearRotorWakeFraction, true, tandemInducedPowerFactor, rearRotorThrustFactor, rearRotorYFilterTc, static_cast<int>(EXT_TRspin), RotorSkewOutput::BlendWithStoredFuselageSkew };

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

	frontCollectiveDeg = 0.0;
	rearCollectiveDeg = 0.0;
	QMRFront = 0.0;
	QMRRear = 0.0;
}
	
void CH46DAero::InitializeOff()
{
	Omega = 0.0;
	OmegaRR = 0.0;
	OmegaE = 0.0;
	PsiMR = 0.0;
	PsiRR = 0.0;
	LambdaMR = 0.0;
	LambdaRR = 0.0;
	CTA = 0.01;
	CTARR = 0.01;
	DWMR = 0.01;
	DWRR = 0.01;
	QMRFront = 0.0;
	QMRRear = 0.0;
	for (int b = 0; b < NUM_BLADES; ++b)
	{
		Beta[b] = 0.0;
		BetaDot[b] = 0.0;
		BetaDotDot[b] = 0.0;
		Delt[b] = 0.0;
		DeltDot[b] = 0.0;
		DeltDotDot[b] = 0.0;
		BetaRR[b] = 0.0;
		BetaDotRR[b] = 0.0;
		BetaDotDotRR[b] = 0.0;
		rearRotorAvgZBSmoothed = 0.0;
		DeltRR[b] = 0.0;
		DeltDotRR[b] = 0.0;
		DeltDotDotRR[b] = 0.0;
		for (int s = 0; s < NUM_BLADE_SEGMENTS; ++s)
		{
			UP[b][s] = 0.0;
			UT[b][s] = 0.0;
			UR[b][s] = 0.0;
			UPRR[b][s] = 0.0;
			UTRR[b][s] = 0.0;
			URRR[b][s] = 0.0;
		}
	}
}

void CH46DAero::InitializeOn()
{
	InitializeOff();
	Omega = OmegaT;
	OmegaRR = OmegaT;
	OmegaE = OmegaT;
	LambdaMR = -0.022;
	LambdaRR = -0.022;
	CTA = 0.014;
	CTARR = 0.014;
	DWMR = 0.022;
	DWRR = 0.022;
}

void CH46DAero::update(double engtorque)
{
	double thetaFront = 0.0;
	double thetaRear = 0.0;
	double a1Front = 0.0;
	double a1Rear = 0.0;
	double b1Front = 0.0;
	double b1Rear = 0.0;
	TandemRotorControlMix(thetaFront, thetaRear, a1Front, a1Rear, b1Front, b1Rear);
#if CH46_LOG_PRE_ROTOR
	{
		char buf[384];
		snprintf(buf, sizeof(buf),
			"[CH46 pre-rotor] t=%.4f Q_eng=%.1f [ft-lb] stick R/P=%.4f/%.4f rudder=%.4f coll=%.4f | out R/P=%.4f/%.4f thetaF/R=%.2f/%.2f a1F/R=%.2f/%.2f b1F/R=%.2f/%.2f\n",
			p_EFMdata.time,
			engtorque,
			p_flightControl.RollInput,
			p_flightControl.PitchInput,
			p_flightControl.PedalInput,
			p_flightControl.CollectiveInput,
			p_flightControl.rollOutput,
			p_flightControl.pitchOutput,
			thetaFront,
			thetaRear,
			a1Front,
			a1Rear,
			b1Front,
			b1Rear);
		OutputDebugStringA(buf);
	}
#endif
	frontCollectiveDeg = thetaFront;
	rearCollectiveDeg = thetaRear;
	advanceRotorDisk(RotorId::Front, rotorCfgFront_, thetaFront, a1Front, b1Front, 0.0);
	advanceRotorDisk(RotorId::Rear, rotorCfgRear_, thetaRear, a1Rear, b1Rear, DWMR);
	QMR = abs(QMRFront) + abs(QMRRear);
	FuselageModule();
	EmpennageModule();
	RotorDegreeOfFreedom(engtorque);
}

void CH46DAero::TandemRotorControlMix(double& thetaFront, double& thetaRear,
	double& a1Front, double& a1Rear,
	double& b1Front, double& b1Rear) const
{
	// Scaffold control mixing for tandem rotors:
	// - yaw input uses differential collective
	// - pitch input adds a small fore/aft collective split
	// - roll/cyclic is fed to both rotors
	const double baseCollectiveDeg = p_flightControl.CollectiveInput * 19.0 + 2.0;
	const double frontRearBias = 0.5 * centeringDCP;
	const double pedalSplitDeg = p_flightControl.PedalInput * rearCollectiveAuthority;
	const double pitchSplitDeg = p_flightControl.pitchOutput * pitchCollectiveBias;

	thetaFront = baseCollectiveDeg - frontRearBias + pedalSplitDeg - pitchSplitDeg;
	thetaRear = baseCollectiveDeg + frontRearBias - pedalSplitDeg + pitchSplitDeg;

	a1Front = p_flightControl.rollOutput * 8.0;
	a1Rear = p_flightControl.rollOutput * 8.0;
	b1Front = p_flightControl.pitchOutput * 12.0;
	b1Rear = p_flightControl.pitchOutput * 12.0;
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
	double BetaF = asin(VYF / sqrt(VXF * VXF + VYF * VYF + VZF * VZF));//angle of sideslip fuselage, [rad]
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

void CH46DAero::EmpennageModule()
{
	/*
	todo: need to change wash factor to just be effective at high wake angles
	*/


	//EKTXU = _EKTXU(limit(skewAngleMR, -20, 100));
	//EKTZU = _EKTZU(limit(skewAngleMR, -20, 100));

	//double epsilonWT;//data tables***
	//double epsilonWTU = _EpsilonWTU(limit(alphaFdeg, -8, 28));//wing downwash angle at upper HT, [deg]
	//double VZIW = (epsilonWT / 57.3) * VXB;
	//VZIWU = (epsilonWTU / 57.3) * VXB;

	// Sidewash
	//double sigmaWT = _SigmaWT(limit(abs(-PsiFdeg), 0, 60));//wing sidewash angle at VT, [deg]
	//VYIW = (sigmaWT / 57.3) * VXB;

	//double TauT = lHT / VXB;
	//TauTU = lHTU / VXB;

	//-----------------------------------------
	//		 Horizontal tail
	//-----------------------------------------
	double lHT = (FSHT - FSCG) / 12.0;//horizontal tail moment arm x axis, [ft]
	double hHT = (WLHT - WLCG) / 12.0;//horizontal tail moment arm z axis, [ft]

	double KQHT;//dynamic pressure loss factor HT
	if (alphaF_deg >= 10.0 && alphaF_deg <= 46.0)
	{
		KQHT = sqrt((0.4 + 0.0333 * abs(alphaF_deg - 28.0)));
	}
	else
	{
		KQHT = 1.0;
	}

	double VXHT = VXB * KQHT - q * hHT;// +EKTXU * (DWMR * OmegaT * RMR);//HT x body axis velocity, [ft/s]
	double VYHT = VYB - r * lHT + p * hHT;// - VYIW* TauTU;//HT y body axis velocity, [ft/s]
	double VZHT = VZB + q * lHT;// -VZIWU * TauTU - EKTZU * (DWMR * OmegaT * RMR);//HT z body axis velocity, [ft/s]
	double VHT2 = VXHT * VXHT + VYHT * VYHT + VZHT * VZHT;
	double qHT = 0.5 * p_EFMdata.rho_SlgFt3 * VHT2;//dynamic pressure at HT, [lb/ft^2]

	double alphaHT = atan2(VZHT, VXHT);//Horiz Tail AoA, [rad]
	double alphaHTT = alphaHT * Convert::radToDeg + iHT;//total AoA of HT, [deg]

	double CL_HT = 0.0;// fn_CL_NACA0015.interpnf1(abs(alphaHTT));
	double CD_HT = 0.0;// fn_CD_NACA0015.interpnf1(abs(alphaHTT));
	double machHT = sqrt(VHT2) / p_EFMdata.speedOfSound_fts;
	//if (abs(alphaHTT) <= 30.0)
	//{
	//	CL_HT = _CL_NACA0015_30(abs(alphaHTT),machHT);
	//	CD_HT = _CD_NACA0015_30(abs(alphaHTT), machHT);
	//}
	//else
	//{
	CL_HT = fn_CL_NACA0015.interpnf1(abs(alphaHTT));
	CD_HT = fn_CD_NACA0015.interpnf1(abs(alphaHTT));
	//}
	if (alphaHTT < 0)
	{
		CL_HT = -CL_HT;
	}
	//double CD_HT = fn_CD_NACA0015.interpnf1(abs(alphaHTT));

	double XHTU = -(CD_HT * cos(alphaHT) * cos(alphaVT) - CL_HT * sin(alphaHT)) * SAHT * qHT;//HT longitudinal force, [lb]
	double YHTU = -(CD_HT * sin(alphaVT)) * qHT;//HT lateral force, lb
	double ZHTU = -(CD_HT * sin(alphaHT) * cos(alphaVT) + CL_HT * cos(alphaHT)) * SAHT * qHT;//HT vertical force, [lb]

	ForceComponent HTForce;
	HTForce.dir.x = XHTU * Convert::lbf_to_N;
	HTForce.dir.y = -ZHTU * Convert::lbf_to_N;
	HTForce.dir.z = YHTU * Convert::lbf_to_N;
	HTForce.pos.x = -lHT * Convert::feetToMeter;
	HTForce.pos.y = hHT * Convert::feetToMeter;
	HTForce.pos.z = 0.0;
	aeroForces.push_back(HTForce);

	//--------------------------------------
	//		 vertical tail 
	//--------------------------------------
	double lVT = (FSVT - FSCG) / 12.0;//vertical tail moment arm x axis, [ft]
	double hVT = (WLVT - WLCG) / 12.0;//vertical tail moment arm z axis, [ft]

	double qVTqBar;
	if (PsiFdeg >= -17.5 && PsiFdeg <= 17.5)
	{
		qVTqBar = 0.68 + 0.01314 * abs(PsiFdeg);
	}
	else if (PsiFdeg < -17.5)
	{
		qVTqBar = 0.91 - 0.00124 * (PsiFdeg + 17.5);
	}
	else if (PsiFdeg > 17.5)
	{
		qVTqBar = 0.91 + 0.00124 * (PsiFdeg - 17.5);
	}
	//double qVTqBarAS = _qVTqBar(limit(PsiFdeg, 0, 30), limit(abs(alphaFdeg), -20, 20));
	KQVT = 1.0;// pow(qVTqBar + qVTqBarAS, 0.5);


	double VXVT = VXB * KQVT - q * hVT;// +EKTXU * (DWMR * OmegaT * RMR);//VT x axis velocity, [ft/sec]
	double VYVT = VYB - r * lVT + p * hVT;// -VYIW * TauTU + EKTR * (DWMR * OmegaT * RMR);//VT y axis velocity, [ft/sec]
	double VZVT = VZB + q * lVT;// -VZIWU * TauTU - EKTZU * (DWMR * OmegaT * RMR);//VT z axis velocity, [ft/sec]
	double VVT2 = VXVT * VXVT + VYVT * VYVT + VZVT * VZVT;
	double qVT = 0.5 * p_EFMdata.rho_SlgFt3 * VVT2;// dynamic presure at VT

	alphaVT = asin(VYVT / sqrt(VVT2));//[rad]
	double alphaVTT = alphaVT * Convert::radToDeg;//AoA VT, [deg]

	double CL_VT = 0.0;// fn_CL_NACA0015.interpnf1(abs(alphaVTT));
	double CD_VT = 0.0;// fn_CD_NACA0015.interpnf1(abs(alphaVTT));
	double machVT = sqrt(VVT2) / p_EFMdata.speedOfSound_fts;
	//if (abs(alphaVTT) <= 30.0)
	//{
	//	CL_VT = _CL_NACA0015_30(abs(alphaVTT), machVT);
	//	CD_VT = _CD_NACA0015_30(abs(alphaVTT), machVT);
	//}
	//else
	//{
	CL_VT = fn_CL_NACA0015.interpnf1(abs(alphaVTT));
	CD_VT = fn_CD_NACA0015.interpnf1(abs(alphaVTT));
	//}
	if (alphaVTT < 0)
	{
		CL_VT = -CL_VT;
	}


	double XVT = -(CD_VT * cos(alphaVT) - CL_VT * sin(alphaVT)) * SAVT * qVT;//VT longitudinal force, [lb]
	double YVT = -(CD_VT * sin(alphaVT) + CL_VT * cos(alphaVT)) * SAVT * qVT;//VT lateral force, [lb]
	double ZVT = 0;//VT vertical force, [lb]

	ForceComponent VTForce;
	VTForce.dir.x = XVT * Convert::lbf_to_N;
	VTForce.dir.y = -ZVT * Convert::lbf_to_N;
	VTForce.dir.z = YVT * Convert::lbf_to_N;
	VTForce.pos.x = -lVT * Convert::feetToMeter;
	VTForce.pos.y = hVT * Convert::feetToMeter;
	VTForce.pos.z = 0;
	aeroForces.push_back(VTForce);
}


// todo: add TR torque?, tune inertias/ratios
void CH46DAero::RotorDegreeOfFreedom(double engtorque)
{
	/* Summary of operation
	* Change in collective causes change in MR torque which changes the speed of the rotor, if engine torque is constant.
	* Engine governor tries to keep constant N2 speed, therefore with a change in N2 speed,
	* the governor will change fuel flow, which changes N1, which changes N2 speed and engine torque output.
	* this change in engine torque will correct rotor speed then match required torque of MR
	* // OmegaDot (angular accel)=torque/inertia
	*/


	// torque "required/used" by main rotor	
	QMRfiltered = QMRfiltered * ((KFRQ - 1.0) / KFRQ) + (1.0 / KFRQ) * QMR;//filtered torque (required to maintain constant rpm) from main rotor
	double rotorBrakeTorque = 0.0;
	if (isRotorBrakeEngaged)// additional torque from rotor brake to help MR stop sooner
	{
		rotorBrakeTorque = LinInterp(Omega, OmegaT * 0.4, 0.0, 200, 500);
	}
	double netMRTorque = -QMRfiltered - rotorBrakeTorque;

	// torque produced by engine pre-clutch
	double QEng = engtorque; // torque produced by N1 gas producer
	double N2torque = LinInterp(OmegaE, 1.0, OmegaT, 25.0, 600.0);// torque needed to drive N2 turbine, [ft-lb], 600 ft-lb ~=100%rpm torque when clutch disengaged
	double netEngTorque = QEng - N2torque;// remaining torque available to turn MR

	//--- Overrunning Clutch ----------------------------------------------------------------

	if (OmegaE > Omega)
	{
		isClutchEngaged = true;
	}
	else
	{
		isClutchEngaged = false;
	}


	if (isClutchEngaged == true)
	{
		netMRTorque += abs(netEngTorque);
		OmegaE = Omega;
	}

	double OmegaEdot = netEngTorque / JE;//engine shaft acceleration, [rad/sec^2]
	OmegaE += OmegaEdot * p_EFMdata.deltaTime;
	OmegaE = limit(OmegaE, 0.0, OmegaT * 1.2);


	double OmegaMRDot = netMRTorque / JMR;//MR acceleration, [rad/s^2].   to disable rotor DOF set to 0
	Omega += OmegaMRDot * p_EFMdata.deltaTime;
	Omega = limit(Omega, 0.0, OmegaT * 1.2);//limit rotor speed to 120% to avoid FM anomalies, it should probably break at that point anyways
	OmegaRR = Omega;


	//----- Outputs for lua indicator script -----
	cockpitAPI.setParamNumber(EFM_N2_RPM, OmegaE / OmegaT * 100.0);
	cockpitAPI.setParamNumber(EFM_NR_RPM, Omega / OmegaT * 100.0);
}