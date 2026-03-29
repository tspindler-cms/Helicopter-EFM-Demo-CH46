#pragma once

#include <cstdint>

/// Identifies which tandem rotor disk is being advanced (front vs rear).
enum class RotorId : std::uint8_t
{
	Front,
	Rear
};

/// How this disk contributes to fuselage wake skew angle (stored in `skewAngleMR`).
enum class RotorSkewOutput : std::uint8_t
{
	/// `skewAngleMR` is replaced with this disk's skew (front rotor).
	ReplaceFuselageSkew,
	/// `skewAngleRR` is set, then `skewAngleMR` becomes average of previous and rear skew.
	BlendWithStoredFuselageSkew
};

/// Per-disk geometry and tandem tuning for `CH46DAero::advanceRotorDisk`.
struct RotorDiskConfig
{
	double hubFS_in = 0.0;
	double hubWL_in = 0.0;
	double hubBL_in = 0.0;
	double shaftTilt_rad = 0.0;
	/// Multiply shaft rate for azimuth / blade equations (+1 front, -1 rear counter-rotation).
	double omegaSign = 1.0;
	/// Extra inflow deficit from the other disk's normalized downwash (0 front, fraction rear).
	double wakeFromOtherDownwash = 0.0;
	bool limitMuTOT = false;
	/// Scales shaft torque from this disk (tandem induced power on rear).
	double torqueInducedFactor = 1.0;
	/// Scales blade body-axis forces before applying to `aeroForces`.
	double thrustScale = 1.0;
	/// 0: per-blade body Y like front; >0: smoothed average Y (rear oscillation fix).
	double yBodyFilterTimeConst = 0.0;
	int extSpinArg = 0;
	RotorSkewOutput skewPolicy = RotorSkewOutput::ReplaceFuselageSkew;
};
