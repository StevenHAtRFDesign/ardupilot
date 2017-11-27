/*
 * level.cpp
 *
 *  Created on: 25Oct.,2017
 *      Author: snh
 */

#include <stdio.h>
#include "level.h"

#define LEVEL_DEFAULT_ACCEL_CMSS 1

Level::Level(void)
{
	_MaxPitchRoll = 1;
	_RollPreTarget = 0;
	_PitchPreTarget = 0;
	_pChannel = NULL;
	_pAccel = NULL;
}

/*
 * Input the pitch and roll targets.  These are converted to motor tilt and output pitch and roll, depending on the
 * mix.
 */
void Level::InputPitchRollPreTargets(float Pitch, float Roll)
{
	_PitchPreTarget = Pitch;
	_RollPreTarget = Roll;
}

/*
 * Input the pitch and roll targets, use the given attitude controller to get the max lean angle,
 * and output the forward and lateral targets to the given motors.
 *
 * Inputs:	Pitch - The input pitch.  Arbitrary units as long as they are the same as the max lean angle
 * 						given by the attitude controller.
 * 			Roll - The input roll.  Arbitrary units as long as they are the same as the max lean angle
 * 						given by the attitude controller.
 * 			pAC - The attitude controller.  Must not be NULL.
 * 			pMotors - The motors to output forward and lateral targets to.  Must not be NULL.
 */
void Level::InputPitchRollPreTargets(float Pitch, float Roll, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputPitchRollPreTargets(Pitch, Roll);
	InputMaxPitchRoll(pAC->lean_angle_max());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

/*
 * Set the level mix RC channel.
 *
 * Inputs:	pChannel - The level mix RC channel.  Can be NULL if not using level flight.
 */
void Level::SetRCChannel(RC_Channel *pChannel)
{
	_pChannel = pChannel;
}

/*
 * Input the maximum pitch and roll angles.
 *
 * Inputs:	MaxPitchRoll - The maximum pitch and roll angle.  Units must be the same as the input
 * 							pitch and roll.
 */
void Level::InputMaxPitchRoll(float MaxPitchRoll)
{
	_MaxPitchRoll = MaxPitchRoll;
}

/*
 * Input the pitch and roll targets and the maximum pitch and roll, and output forward
 * and lateral to the given motors.
 *
 * Inputs:	Pitch - The input pitch.  Arbitrary units as long as they are the same as the max lean angle
 * 						given by the attitude controller.
 * 			Roll - The input roll.  Arbitrary units as long as they are the same as the max lean angle
 * 						given by the attitude controller.
 * 			Max - The maximum pitch and roll angle.  Units must be the same as the input
 * 							pitch and roll.
 * 			pMotors - The motors to output forward and lateral targets to.  Must not be NULL.
 */
void Level::InputPitchRollMax(float Pitch, float Roll, float Max, AP_Motors *pMotors)
{
	InputPitchRollPreTargets(Pitch, Roll);
	InputMaxPitchRoll(Max);
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

/*
 * Input target pitch and roll data from the given AC_WPNav, use max lean angle from the
 * given attitude controller and output forward and lateral targets to the given motors
 *
 * Inputs:	pNav - The AC_WPNav.  Must not be NULL.
 * 			pAC - The attitude controller.  Must not be NULL.
 * 			pMotors - The motors.  Must not be NULL.
 */
void Level::InputFromWPNav(AC_WPNav *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputMaxPitchRoll(pAC->lean_angle_max());
	InputPitchRollPreTargets(pNav->get_pitch(), pNav->get_roll());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

/*
 * Input target pitch and roll data from the given AC_Circle, use max lean angle from the
 * given attitude controller and output forward and lateral targets to the given motors
 *
 * Inputs:	pNav - The AC_Circle.  Must not be NULL.
 * 			pAC - The attitude controller.  Must not be NULL.
 * 			pMotors - The motors.  Must not be NULL.
 */
void Level::InputFromCircleNav(AC_Circle *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputMaxPitchRoll(pAC->lean_angle_max());
	InputPitchRollPreTargets(pNav->get_pitch(), pNav->get_roll());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

/*
 * Input target pitch and roll data from the given AC_PosControl, use max lean angle from the
 * given attitude controller and output forward and lateral targets to the given motors
 *
 * Inputs:	pNav - The AC_PosControl.  Must not be NULL.
 * 			pAC - The attitude controller.  Must not be NULL.
 * 			pMotors - The motors.  Must not be NULL.
 */
void Level::InputFromPosControl(AC_PosControl *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputMaxPitchRoll(pAC->lean_angle_max());
	InputPitchRollPreTargets(pNav->get_pitch(), pNav->get_roll());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

/*
 * Returns the motor forward target.
 */
float Level::GetForwardTarget(void)
{
	return -GetMix() * _PitchPreTarget / _MaxPitchRoll;
}

/*
 * Returns the motor lateral target.
 */
float Level::GetLateralTarget(void)
{
	return GetMix() * _RollPreTarget / _MaxPitchRoll;
}

/*
 * Returns the body pitch target.
 */
float Level::GetPitchTarget(void)
{
	return (1 - GetMix()) * _PitchPreTarget;
}

/*
 * Returns the body roll target.
 */
float Level::GetRollTarget(void)
{
	return (1 - GetMix()) * _RollPreTarget;
}

/*
 * Returns the level mix fraction.
 */
float Level::GetMix(void)
{
	if (_pChannel == NULL)
	{
		return 0;
	}
	else
	{
		//printf("Mix ch %d\n", _pChannel->get_control_in());
		return (float)(_pChannel->get_control_in()) / 1000.0;
	}
}

float Level::ModifyAcceleration(float A)
{

	float Mix = GetMix();
	return (Mix * GetLevelAccel()) + ((1 - Mix) * A);
}

void Level::SetParameter(AP_Float *pAccel)
{
	_pAccel = pAccel;
}

float Level::GetLevelAccel(void)
{
	if (_pAccel == NULL)
	{
		return LEVEL_DEFAULT_ACCEL_CMSS;
	}
	else
	{
		return _pAccel->get();
	}
}

void LevelModifyAcceleration::SetLevel(Level *pL)
{
	_pLevel = pL;
}

float LevelModifyAcceleration::Function(float A)
{
	return _pLevel->ModifyAcceleration(A);
}
