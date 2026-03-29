#include "../stdafx.h"
#include "Aero.h"
#include <cmath>

void CH46DAero::advanceRotorDisk(RotorId id, const RotorDiskConfig& cfg, double thetaC, double a1, double b1, double dwOtherNormalized)
{
	const bool isFront = (id == RotorId::Front);
	double& psiHub = isFront ? PsiMR : PsiRR;
	double& lambda = isFront ? LambdaMR : LambdaRR;
	double& dw = isFront ? DWMR : DWRR;
	double& ctRef = isFront ? CTA : CTARR;

	double* psiB = isFront ? Psi : PsiRR_blades;
	double* sinPsiB = isFront ? SinPsi : SinPsiRR;
	double* cosPsiB = isFront ? CosPsi : CosPsiRR;
	double* beta = isFront ? Beta : BetaRR;
	double* betaDot = isFront ? BetaDot : BetaDotRR;
	double* betaDotDot = isFront ? BetaDotDot : BetaDotDotRR;
	double* delt = isFront ? Delt : DeltRR;
	double* deltDot = isFront ? DeltDot : DeltDotRR;
	double* deltDotDot = isFront ? DeltDotDot : DeltDotDotRR;
	double* sinBeta = isFront ? SinBeta : SinBetaRR;
	double* cosBeta = isFront ? CosBeta : CosBetaRR;
	double* sinDelt = isFront ? SinDelt : SinDeltRR;
	double* cosDelt = isFront ? CosDelt : CosDeltRR;
	double (*up)[NUM_BLADE_SEGMENTS] = isFront ? UP : UPRR;
	double (*ut)[NUM_BLADE_SEGMENTS] = isFront ? UT : UTRR;
	double (*ur)[NUM_BLADE_SEGMENTS] = isFront ? UR : URRR;
	double (*segTheta)[NUM_BLADE_SEGMENTS] = isFront ? Theta : ThetaRR;
	double (*aTrans)[NUM_BLADE_SEGMENTS] = isFront ? alphaTrans : alphaTransRR;
	double (*cly)[NUM_BLADE_SEGMENTS] = isFront ? CLY : CLYRR;
	double (*cdy)[NUM_BLADE_SEGMENTS] = isFront ? CDY : CDYRR;
	double (*fp)[NUM_BLADE_SEGMENTS] = isFront ? FP : FPRR;
	double (*ft)[NUM_BLADE_SEGMENTS] = isFront ? FT : FTRR;
	double (*fr)[NUM_BLADE_SEGMENTS] = isFront ? FR : FRRR;
	double* fxa = isFront ? FXA : FXARR;
	double* fya = isFront ? FYA : FYARR;
	double* fza = isFront ? FZA : FZARR;
	double* mfa = isFront ? MFA : MFARR;
	double* mla = isFront ? MLA : MLARR;
	double* mfd = isFront ? MFD : MFDRR;
	double* mld = isFront ? MLD : MLDRR;
	double* fxt = isFront ? FXT : FXTRR;
	double* fyt = isFront ? FYT : FYTRR;
	double* fzt = isFront ? FZT : FZTRR;

	const double iShaft = cfg.shaftTilt_rad;
	const double omegaMag = Omega;
	const double omegaSigned = cfg.omegaSign * omegaMag;

	double Weight = p_EFMdata.mass_kg * Convert::kg_to_lb;
	double Wbd = Weight - NUM_BLADES * Wb;
	double FSCGB = (FSCG * Weight - NUM_BLADES * cfg.hubFS_in * Wb) / Wbd;
	double WLCGB = (WLCG * Weight - NUM_BLADES * cfg.hubWL_in * Wb) / Wbd;
	double BLCGB = (BLCG * Weight - NUM_BLADES * cfg.hubBL_in * Wb) / Wbd;
	double lHub = (FSCGB - cfg.hubFS_in) / 12.0;
	double bHub = (BLCGB - cfg.hubBL_in) / 12.0;
	double hHub = (WLCGB - cfg.hubWL_in) / 12.0;

	double VXHDot = VXBDot - r * VYB + q * VZB - lHub * (q * q + r * r) + bHub * (p * q - rDot) + hHub * (p * r + qDot) + Convert::gravity_fts2 * sin(ThetaB);
	double VYHDot = VYBDot - p * VZB + r * VXB + lHub * (p * q + rDot) - bHub * (p * p + r * r) + hHub * (q * r - pDot) - Convert::gravity_fts2 * sin(PhiB) * cos(ThetaB);
	double VZHDot = VZBDot - q * VXB + p * VYB + lHub * (p * r - qDot) + bHub * (q * r + pDot) - hHub * (p * p + q * q) - Convert::gravity_fts2 * cos(PhiB) * cos(ThetaB);

	double MuXH = (VXB + q * hHub - r * bHub) / (OmegaT * RMR);
	double MuYH = (VYB - p * hHub + r * lHub) / (OmegaT * RMR);
	double MuZH = (VZB - q * lHub + p * bHub) / (OmegaT * RMR);

	double VXSDot = VXHDot * cos(iShaft) - VZHDot * sin(iShaft);
	double VYSDot = VYHDot;
	double VZSDot = VXHDot * sin(iShaft) + VZHDot * cos(iShaft);
	double pDotS = pDot * cos(iShaft) - rDot * sin(iShaft);
	double qDotS = qDot;
	double rDotS = pDot * sin(iShaft) + rDot * cos(iShaft);
	double MuXS = MuXH * cos(iShaft) - MuZH * sin(iShaft);
	double MuYS = MuYH;
	double MuZS = MuXH * sin(iShaft) + MuZH * cos(iShaft);
	double pS = p * cos(iShaft) - r * sin(iShaft);
	double qS = q;
	double rS = p * sin(iShaft) + r * cos(iShaft);

	double MuTOT = sqrt(MuXS * MuXS + MuYS * MuYS + lambda * lambda);
	if (cfg.limitMuTOT)
	{
		MuTOT = limit(MuTOT, 0.0001, 100.0);
	}
	double KGL = sqrt(pow(MuXS, 2) + pow(MuYS, 2)) / MuTOT;
	double K1X = KGL * (MuXS / MuTOT);
	double K1Y = KGL * (MuYS / MuTOT);

	double Klambda = KlambdaPrime / p_EFMdata.deltaTime;
	dw = ((Klambda - 1.0) / Klambda) * dw + (1.0 / Klambda) * (ctRef / MuTOT);
	lambda = MuZS - dw - cfg.wakeFromOtherDownwash * dwOtherNormalized;

	double DeltaPsi = omegaSigned * p_EFMdata.deltaTime;
	psiHub += DeltaPsi;
	while (psiHub > M_PI)
	{
		psiHub -= 2.0 * M_PI;
	}
	while (psiHub < -M_PI)
	{
		psiHub += 2.0 * M_PI;
	}

	psiB[0] = psiHub;
	for (int b = 1; b < NUM_BLADES; b++)
	{
		psiB[b] = psiB[b - 1] + 2.0 * M_PI / NUM_BLADES;
		if (psiB[b] > M_PI)
		{
			psiB[b] -= 2.0 * M_PI;
		}
	}

	double Z_rotor = limit(p_EFMdata.altitudeAGL_ft + (cfg.hubWL_in - WLCG) / 12.0, 1.0, 1000.0);
	double Kge = limit(pow(1.0 + 0.13 * pow(RMR / Z_rotor, 2) * lambda / sqrt(pow(MuXS, 2) + pow(MuYS, 2) + pow(lambda, 2)), -2.0 / 3.0), 1.0, 1.5);

	for (int b = 0; b < NUM_BLADES; b++)
	{
		sinPsiB[b] = sin(psiB[b]);
		cosPsiB[b] = cos(psiB[b]);

		double OmegaB = limit(fabs(omegaSigned), OmegaT / 10.0, OmegaT * 5.0);
		beta[b] = beta[b] + betaDot[b] * sin(DeltaPsi) / OmegaB + betaDotDot[b] * (1.0 - cos(DeltaPsi)) / pow(OmegaB, 2);
		beta[b] = limit(beta[b], betaDown, betaUp);

		betaDot[b] = betaDot[b] * cos(DeltaPsi) + betaDotDot[b] * sin(DeltaPsi) / OmegaB;
		betaDot[b] = limit(betaDot[b], -50.0, 50.0);

		betaDotDot[b] = (Mb / Ib) * (cos(beta[b]) * (VZSDot + e * (2.0 * omegaSigned * (pS * cosPsiB[b] - qS * sinPsiB[b]) + pDotS * sinPsiB[b] + qDotS * cosPsiB[b])) + sin(beta[b]) * cos(delt[b]) * (VYSDot * sinPsiB[b] - VXSDot * cosPsiB[b] - e * pow(omegaSigned - rS, 2)) + sin(delt[b]) * sin(beta[b]) * (VXSDot * sinPsiB[b] + VYSDot * cosPsiB[b] - e * (rDotS - OmegaDot)))
			+ pow(cos(beta[b]), 2) * (cos(delt[b]) * (pDotS * sinPsiB[b] + qDotS * cosPsiB[b] - 2.0 * (deltDot[b] + omegaSigned) * (qS * sinPsiB[b] - pS * cosPsiB[b])) - sin(delt[b]) * (2.0 * (omegaSigned + deltDot[b]) * (pS * sinPsiB[b] + qS * cosPsiB[b]) + qDotS * sinPsiB[b] - pDotS * cosPsiB[b]))
			+ cos(beta[b]) * sin(beta[b]) * (2.0 * deltDot[b] * (rS - omegaSigned) - pow(omegaSigned - rS, 2) - pow(deltDot[b], 2))
			+ (mfa[b] / Ib) + (mfd[b] / Ib);
		betaDotDot[b] = limit(betaDotDot[b], -2000.0, 2000.0);

		sinBeta[b] = sin(beta[b]);
		cosBeta[b] = cos(beta[b]);

#ifdef USE_LAG_DOF
		delt[b] = delt[b] + deltDot[b] * sin(DeltaPsi) / OmegaB + deltDotDot[b] * (1.0 - cos(DeltaPsi)) / pow(OmegaB, 2);
		delt[b] = limit(delt[b], deltAFT, deltFWD);

		deltDot[b] = deltDot[b] * cos(DeltaPsi) + deltDotDot[b] * sin(DeltaPsi) / OmegaB;
		deltDot[b] = limit(deltDot[b], -500.0, 500.0);

		deltDotDot[b] = (Mb / (Ib * cos(beta[b]))) * (sin(delt[b]) * (VYSDot * sinPsiB[b] - VXSDot * cosPsiB[b] - e * pow(omegaSigned - rS, 2)) - cos(delt[b]) * (VXSDot * sinPsiB[b] + VYSDot * cosPsiB[b] + e * (OmegaDot - rDotS)))
			+ (sin(beta[b]) / cos(beta[b])) * (2.0 * betaDot[b] * (omegaSigned + deltDot[b] - rS) + qDotS * sin(psiB[b] + delt[b]) - pDotS * cos(psiB[b] + delt[b])) + (rDotS - OmegaDot)
			+ 2.0 * betaDot[b] * (cos(delt[b]) * (qS * sinPsiB[b] - pS * cosPsiB[b]) + sin(delt[b]) * (pS * sinPsiB[b] + qS * cosPsiB[b]))
			- mla[b] / (Ib * cos(beta[b])) - mld[b] / (Ib * cos(beta[b]));
		deltDotDot[b] = limit(deltDotDot[b], -2000.0, 2000.0);
#endif
		sinDelt[b] = sin(delt[b]);
		cosDelt[b] = cos(delt[b]);

		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			up[b][s] = lambda * cosBeta[b] + MuYS * sinBeta[b] * sin(psiB[b] + delt[b]) - MuXS * sinBeta[b] * cos(psiB[b] + delt[b])
				+ XI * cosBeta[b] * ((qS / OmegaT - K1X * dw) * cosPsiB[b] + (pS / OmegaT + K1Y * dw) * sinPsiB[b])
				+ XI * sinBeta[b] * sinDelt[b] * ((omegaSigned - rS) / OmegaT)
				+ Y2[s] * (-(betaDot[b] / OmegaT) + (qS / OmegaT - K1X * dw * cosBeta[b]) * cos(psiB[b] + delt[b]) + (pS / OmegaT + K1Y * dw * cosBeta[b]) * sin(psiB[b] + delt[b]));

			ut[b][s] = MuXS * sin(psiB[b] + delt[b]) + MuYS * cos(psiB[b] + delt[b])
				+ XI * cosDelt[b] * ((omegaSigned - rS) / OmegaT)
				+ Y2[s] * ((deltDot[b] / OmegaT) + (pS / OmegaT * cos(psiB[b] + delt[b]) - qS / OmegaT * sin(psiB[b] + delt[b])) * sinBeta[b] + cosBeta[b] * ((omegaSigned - rS) / OmegaT));
			if (ut[b][s] == 0.0) { ut[b][s] = 0.00001; }

			ur[b][s] = lambda * sinBeta[b] + MuXS * cosBeta[b] * cos(psiB[b] + delt[b]) - MuYS * cosBeta[b] * sin(psiB[b] + delt[b])
				+ XI * sinBeta[b] * ((qS / OmegaT - K1X * dw) * cosPsiB[b] + (pS / OmegaT + K1Y * dw) * sinPsiB[b])
				- XI * cosBeta[b] * sinDelt[b] * ((omegaSigned - rS) / OmegaT)
				+ Y2[s] * sinBeta[b] * (-K1X * dw * cos(psiB[b] + delt[b]) + K1Y * dw * sin(psiB[b] + delt[b]));

			double UYAW = sqrt(pow(ut[b][s], 2) + pow(up[b][s], 2) + pow(ur[b][s], 2));
			double cosGamma = abs(ut[b][s]) / sqrt(pow(ut[b][s], 2) + pow(ur[b][s], 2));
			if (cosGamma == 0.0) { cosGamma = 0.00001; }

			segTheta[b][s] = (thetaC - a1 * cos(psiB[b] + DeltaSP) - b1 * sin(psiB[b] + DeltaSP) + Theta1 * (Y2[s] - XIPrime) - 57.3 * beta[b] * tan(delt3)) * Convert::degToRad;

			double alphaY = atan2((ut[b][s] * tan(segTheta[b][s]) + up[b][s]) * cosGamma, ut[b][s] - up[b][s] * tan(segTheta[b][s]) * pow(cosGamma, 2)) * Convert::radToDeg;
			alphaY = limit(alphaY, -180.0, 180.0);

			if (abs(alphaY) >= 0 && abs(alphaY) <= 13.5 / cosGamma)
			{
				if (abs(alphaY) > 90.0)
				{
					aTrans[b][s] = abs(alphaY * cosGamma + alphaY / abs(alphaY) * 180.0 * (1.0 - cosGamma));
				}
				else
				{
					aTrans[b][s] = abs(alphaY * cosGamma);
				}
			}
			else if (abs(alphaY) >= (180.0 - 8.0 / cosGamma) && abs(alphaY) <= 180.0)
			{
				if (abs(alphaY) > 90.0)
				{
					aTrans[b][s] = abs(alphaY * cosGamma + alphaY / abs(alphaY) * 180.0 * (1.0 - cosGamma));
				}
				else
				{
					aTrans[b][s] = abs(alphaY * cosGamma);
				}
			}
			else if (abs(alphaY) >= 13.5 / cosGamma && abs(alphaY) <= (180.0 - 8.0 / cosGamma))
			{
				aTrans[b][s] = abs(alphaY);
			}
			aTrans[b][s] = limit(aTrans[b][s], 0.0, 180.0);

			double CLfinal = fn_CL_NACA0015.interpnf1(aTrans[b][s]);
			double CDfinal = fn_CD_NACA0015.interpnf1(abs(alphaY));
			if (s < NUM_BLADE_SEGMENTS - 1)
			{
				cly[b][s] = CLfinal * 1.0;
			}
			if (s == NUM_BLADE_SEGMENTS - 1)
			{
				cly[b][s] = (1.0 - ((1.0 - BMR) / DeltaY[NUM_BLADE_SEGMENTS - 1])) * CLfinal;
			}
			if (alphaY < 0.0)
			{
				cly[b][s] = -cly[b][s];
			}
			cdy[b][s] = CDfinal;

			fp[b][s] = 0.5 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 3) * (CR * DeltaY[s]) * UYAW * (cly[b][s] * (ut[b][s] / cosGamma) + cdy[b][s] * up[b][s]);
			ft[b][s] = 0.5 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 3) * (CR * DeltaY[s]) * UYAW * (cdy[b][s] * ut[b][s] - cly[b][s] * up[b][s] * cosGamma);
			fr[b][s] = 0.5 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 3) * (CR * DeltaY[s]) * UYAW * (cdy[b][s] - cly[b][s] * (up[b][s] / ut[b][s]) * cosGamma) * ur[b][s];
		}

		double FPB = 0.0;
		double FTB = 0.0;
		double FRB = 0.0;
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			FPB += fp[b][s];
			FTB += ft[b][s];
			FRB += fr[b][s];
		}

		fxa[b] = FRB * cosBeta[b] * sinDelt[b] - FTB * cosDelt[b] - FPB * sinBeta[b] * sinDelt[b];
		fya[b] = FRB * cosBeta[b] * cosDelt[b] + FTB * sinDelt[b] - FPB * sinBeta[b] * cosDelt[b];
		fza[b] = -(FRB * sinBeta[b] + FPB * cosBeta[b]);

		mfa[b] = 0.0;
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			mfa[b] += Y2[s] * fp[b][s];
		}
		mfa[b] = mfa[b] * RMR;

		mla[b] = 0.0;
		for (int s = 0; s < NUM_BLADE_SEGMENTS; s++)
		{
			mla[b] += Y2[s] * ft[b][s];
		}
		mla[b] = mla[b] * RMR;

		mfd[b] = -(Kbeta * beta[b] + KbetaDot * betaDot[b]);

		double FXI = Mb * (cosBeta[b] * cosDelt[b] * (rDotS - OmegaDot - deltDotDot[b])
			+ 2.0 * sinBeta[b] * cosDelt[b] * (deltDot[b] * betaDot[b] + (omegaSigned - rS) * betaDot[b])
			+ cosBeta[b] * sinDelt[b] * (pow(deltDot[b], 2) + pow(betaDot[b], 2) + 2.0 * (omegaSigned - rS) * deltDot[b] + pow(omegaSigned - rS, 2))
			+ 2.0 * betaDot[b] * cosBeta[b] * (pS * cosPsiB[b] - qS * sinPsiB[b])
			+ betaDotDot[b] * sinBeta[b] * sinDelt[b])
			- (Wb / Convert::gravity_fts2) * (VXSDot * sinPsiB[b] + VYSDot * cosPsiB[b]);
		double FYI = Mb * (cosBeta[b] * cosDelt[b] * (pow(deltDot[b], 2) + pow(betaDot[b], 2) + 2.0 * (omegaSigned - rS) * deltDot[b] + pow(omegaSigned - rS, 2))
			+ betaDotDot[b] * sinBeta[b] * cosDelt[b] + deltDotDot[b] * cosBeta[b] * sinDelt[b] - 2.0 * betaDot[b] * cosBeta[b] * (pS * sinPsiB[b] + qS * cosPsiB[b])
			+ (Wb * e / (Convert::gravity_fts2 * Mb)) * pow(omegaSigned - rS, 2))
			+ (Wb / Convert::gravity_fts2) * (VXSDot * cosPsiB[b] - VYSDot * sinPsiB[b]);
		double FZI = Mb * (betaDotDot[b] * cosBeta[b] - pow(betaDot[b], 2) * sinBeta[b] + 2.0 * betaDot[b] * sinBeta[b] * cosDelt[b] * (pS * sinPsiB[b] + qS * cosPsiB[b])
			+ cosBeta[b] * sinDelt[b] * (2.0 * (omegaSigned + deltDot[b]) * (pS * sinPsiB[b] + qS * cosPsiB[b]) + qDotS * sinPsiB[b] - pDotS * cosPsiB[b])
			- cosBeta[b] * cosDelt[b] * (2.0 * (omegaSigned + deltDot[b]) * (pS * cosPsiB[b] - qS * sinPsiB[b]) + pDotS * sinPsiB[b] + qDotS * cosPsiB[b])
			- (Wb * e / (Convert::gravity_fts2 * Mb)) * (2.0 * omegaSigned * (pS * cosPsiB[b] - qS * sinPsiB[b]) + pDotS * sinPsiB[b] + qDotS * cosPsiB[b]))
			- (Wb / Convert::gravity_fts2) * VZSDot;

		fxt[b] = fxa[b] + FXI;
		fyt[b] = fya[b] + FYI;
		fzt[b] = (fza[b] + FZI) * Kge;

		double bladeHealth = p_Damage.elementIntegrity[BLADE_1_CENTER + b * 3];
		fxt[b] *= bladeHealth;
		fyt[b] *= bladeHealth;
		fzt[b] *= bladeHealth;

		double XB = -(fyt[b] * cosPsiB[b] - fxt[b] * sinPsiB[b]);
		double YB = (fxt[b] * cosPsiB[b] + fyt[b] * sinPsiB[b]);
		double ZB = fzt[b];
		XB *= cfg.thrustScale;
		YB *= cfg.thrustScale;
		ZB *= cfg.thrustScale;

		ForceComponent bladeForce;
		bladeForce.dir.x = limit(XB * cos(iShaft) + ZB * sin(iShaft), -10000.0, 10000.0) * Convert::lbf_to_N;
		bladeForce.dir.z = limit(YB, -10000.0, 10000.0) * Convert::lbf_to_N;
		bladeForce.pos.x = (lHub - e * cosPsiB[b]) * Convert::feetToMeter;
		bladeForce.pos.y = -hHub * Convert::feetToMeter;
		bladeForce.pos.z = (bHub + e * sinPsiB[b]) * Convert::feetToMeter;

		if (cfg.yBodyFilterTimeConst > 0.0)
		{
			bladeForce.dir.y = 0.0;
		}
		else
		{
			bladeForce.dir.y = limit(-(-XB * sin(iShaft) + ZB * cos(iShaft)), -10000.0, 10000.0) * Convert::lbf_to_N;
		}
		aeroForces.push_back(bladeForce);
	}

	if (cfg.yBodyFilterTimeConst > 0.0)
	{
		double avgZB = 0.0;
		for (int b = 0; b < NUM_BLADES; b++)
			avgZB += fzt[b] * cfg.thrustScale;
		avgZB /= NUM_BLADES;
		const double dt = p_EFMdata.deltaTime;
		const double yFltAlpha = (dt > 0.0 && cfg.yBodyFilterTimeConst > 0.0) ? dt / (cfg.yBodyFilterTimeConst + dt) : 1.0;
		rearRotorAvgZBSmoothed = rearRotorAvgZBSmoothed * (1.0 - yFltAlpha) + avgZB * yFltAlpha;
		double rawY = -rearRotorAvgZBSmoothed * cos(iShaft);
		rawY = fabs(limit(rawY, -10000.0, 10000.0)) * Convert::lbf_to_N;
		size_t base = aeroForces.size() - NUM_BLADES;
		for (size_t i = 0; i < (size_t)NUM_BLADES; i++)
			aeroForces[base + i].dir.y = rawY;
	}

	double TA = 0.0;
	for (int b = 0; b < NUM_BLADES; b++)
	{
		TA += fza[b] * p_Damage.elementIntegrity[BLADE_1_CENTER + b * 3];
	}
	TA = -TA;
	ctRef = TA / (2.0 * p_EFMdata.rho_SlgFt3 * pow(OmegaT, 2) * pow(RMR, 4) * M_PI);

	double LH = 0.0;
	double MH = 0.0;
	double Q = 0.0;
	for (int b = 0; b < NUM_BLADES; b++)
	{
		Q += (e * fxt[b] - mla[b] * cosBeta[b]) * p_Damage.elementIntegrity[BLADE_1_CENTER + b * 3];
	}
	Q = -Q * cfg.torqueInducedFactor;
	if (isFront)
	{
		QMRFront = Q;
	}
	else
	{
		QMRRear = Q;
	}

	double Q2 = 0.0;
	for (int b = 0; b < NUM_BLADES; b++)
	{
		Q2 += (-mla[b] * cosBeta[b]) * p_Damage.elementIntegrity[BLADE_1_CENTER + b * 3];
	}
	Q2 = -Q2 * 0.5;

	if (isFront)
	{
		double Kforce = KfPrime / p_EFMdata.deltaTime;
		LMR = ((Kforce - 1.0) / Kforce) * LMR + (1.0 / Kforce) * (LH * cos(iShaft) + Q2 * sin(iShaft));
		MMR = ((Kforce - 1.0) / Kforce) * MMR + (1.0 / Kforce) * (MH);
		NMR = ((Kforce - 1.0) / Kforce) * NMR + (1.0 / Kforce) * (-LH * sin(iShaft) + Q2 * cos(iShaft));
	}

	double a1SF = 0.0;
	for (int b = 0; b < NUM_BLADES; b++)
	{
		a1SF += beta[b] * cosPsiB[b] * p_Damage.elementIntegrity[BLADE_1_CENTER + b * 3];
	}
	a1SF *= -2.0 / NUM_BLADES * Convert::radToDeg;

	double diskSkewDeg = atan2(MuXS, abs(lambda)) * Convert::radToDeg + a1SF;
	if (cfg.skewPolicy == RotorSkewOutput::ReplaceFuselageSkew)
	{
		skewAngleMR = diskSkewDeg;
	}
	else
	{
		skewAngleRR = diskSkewDeg;
		skewAngleMR = 0.5 * (skewAngleMR + skewAngleRR);
	}

	double animPos = psiHub / (2.0 * M_PI);
	if (animPos < 0.0)
	{
		animPos += 1.0;
	}
	cockpitAPI.setExternalDrawArg(cfg.extSpinArg, (float)animPos);
	if (isFront)
	{
		cockpitAPI.setExternalDrawArg(EXT_RotorDroop, (float)LinInterp(Omega, 0, OmegaT * 0.1, -1.0f, 0.0f));
	}

	if (p_EFMdata.time > p_EFMdata.deltaTime * 2)
	{
		if (isFront)
		{
			Vec3 mrMoment;
			mrMoment.x = limit(LMR, -5000.0, 5000.0) * Convert::lbft_to_Nm;
			mrMoment.y = limit(-NMR, -5000.0, 5000.0) * Convert::lbft_to_Nm;
			mrMoment.z = limit(MMR, -5000.0, 5000.0) * Convert::lbft_to_Nm;
			aeroMoments.push_back(mrMoment);
		}
		else
		{
			Vec3 rrMoment;
			rrMoment.x = limit(Q2 * sin(iShaft), -5000.0, 5000.0) * Convert::lbft_to_Nm;
			rrMoment.y = limit(-Q2 * cos(iShaft), -5000.0, 5000.0) * Convert::lbft_to_Nm;
			rrMoment.z = 0.0;
			aeroMoments.push_back(rrMoment);
		}
	}
}
