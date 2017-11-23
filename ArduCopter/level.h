/*
 * level.h
 *
 *  Created on: 25Oct.,2017
 *      Author: snh
 */

#ifndef ARDUCOPTER_LEVEL_H_
#define ARDUCOPTER_LEVEL_H_


#include "AC_AttitudeControl/AC_AttitudeControl.h"
#include "AP_Motors/AP_Motors_Class.h"
#include "AC_WPNav/AC_WPNav.h"
#include "AC_WPNav/AC_Circle.h"
#include "AC_AttitudeControl/AC_PosControl.h"

/*
 * A class for managing level flight of tilt hexacopter, given pitch and roll inputs
 * and a "mix" input (0 to 1) which controls whether flight should be via whole body pitch and
 * roll (0), or via motor tilt (1).
 */
class Level
{
public:
	Level();
	void InputPitchRollPreTargets(float Pitch, float Roll);
	void InputPitchRollPreTargets(float Pitch, float Roll, AC_AttitudeControl *pAC, AP_Motors *pMotors);
	void InputLevelSetting(float Mix);
	void InputLevelSetting(RC_Channel *pChannel);
	void SetRCChannel(RC_Channel *pChannel);
	void InputMaxPitchRoll(float Max);
	void InputPitchRollMax(float Pitch, float Roll, float Max, AP_Motors *pMotors);
	void InputFromWPNav(AC_WPNav *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors);
	void InputFromCircleNav(AC_Circle *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors);
	void InputFromPosControl(AC_PosControl *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors);
	float GetPitchTarget(void);
	float GetRollTarget(void);
	float GetForwardTarget(void);
	float GetLateralTarget(void);
	float GetVelocityTargetScale(void);
	float ModifyAcceleration(float A);

	static const struct AP_Param::GroupInfo var_info[];
private:
	float _PitchPreTarget;
	float _RollPreTarget;
	float _MaxPitchRoll;
	AP_Float _Accel;
	RC_Channel *_pChannel;
	float GetMix(void);
};

class LevelModifyAcceleration : public AP_Function<float>
{
public:
	void SetLevel(Level *pL);
	virtual float Function(float);
private:
	Level *_pLevel;
};

#endif /* ARDUCOPTER_LEVEL_H_ */
