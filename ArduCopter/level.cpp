/*
 * level.cpp
 *
 *  Created on: 25Oct.,2017
 *      Author: snh
 */

#include <stdio.h>
#include "level.h"

Level::Level(void)
{
	_MaxPitchRoll = 1;
	_RollPreTarget = 0;
	_PitchPreTarget = 0;
	_pChannel = NULL;
}

void Level::InputPitchRollPreTargets(float Pitch, float Roll)
{
	_PitchPreTarget = Pitch;
	_RollPreTarget = Roll;
}

void Level::InputPitchRollPreTargets(float Pitch, float Roll, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputPitchRollPreTargets(Pitch, Roll);
	InputMaxPitchRoll(pAC->lean_angle_max());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

void Level::SetRCChannel(RC_Channel *pChannel)
{
	_pChannel = pChannel;
}

void Level::InputMaxPitchRoll(float MaxPitchRoll)
{
	_MaxPitchRoll = MaxPitchRoll;
}

void Level::InputPitchRollMax(float Pitch, float Roll, float Max, AP_Motors *pMotors)
{
	InputPitchRollPreTargets(Pitch, Roll);
	InputMaxPitchRoll(Max);
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

void Level::InputFromWPNav(AC_WPNav *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputMaxPitchRoll(pAC->lean_angle_max());
	InputPitchRollPreTargets(pNav->get_pitch(), pNav->get_roll());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

void Level::InputFromCircleNav(AC_Circle *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputMaxPitchRoll(pAC->lean_angle_max());
	InputPitchRollPreTargets(pNav->get_pitch(), pNav->get_roll());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

void Level::InputFromPosControl(AC_PosControl *pNav, AC_AttitudeControl *pAC, AP_Motors *pMotors)
{
	InputMaxPitchRoll(pAC->lean_angle_max());
	InputPitchRollPreTargets(pNav->get_pitch(), pNav->get_roll());
	pMotors->set_forward(GetForwardTarget());
	pMotors->set_lateral(GetLateralTarget());
}

float Level::GetForwardTarget(void)
{
	return -GetMix() * _PitchPreTarget / _MaxPitchRoll;
}

float Level::GetLateralTarget(void)
{
	return GetMix() * _RollPreTarget / _MaxPitchRoll;
}

float Level::GetPitchTarget(void)
{
	return (1 - GetMix()) * _PitchPreTarget;
}

float Level::GetRollTarget(void)
{
	return (1 - GetMix()) * _RollPreTarget;
}

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

