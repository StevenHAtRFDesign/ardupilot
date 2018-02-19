/*
 * AP_MotorsTiltHexa.cpp
 *
 *  Created on: 13Sep.,2017
 *      Author: snh
 */

#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsTiltHexa.h"

#define TILT_HEXA_QTY_MOTORS 6

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsTiltHexa::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
	printf("\nInit tilthexa!\n");

    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);
    add_motor_num(AP_MOTORS_MOT_4);
    add_motor_num(AP_MOTORS_MOT_5);
    add_motor_num(AP_MOTORS_MOT_6);

    // set update rate for the 6 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    _motor_servo[AP_MOTORS_MOT_1] = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, AP_MOTORS_MOT_1);
    _motor_servo[AP_MOTORS_MOT_2] = SRV_Channels::get_channel_for(SRV_Channel::k_motor2, AP_MOTORS_MOT_2);
    _motor_servo[AP_MOTORS_MOT_3] = SRV_Channels::get_channel_for(SRV_Channel::k_motor3, AP_MOTORS_MOT_3);
    _motor_servo[AP_MOTORS_MOT_4] = SRV_Channels::get_channel_for(SRV_Channel::k_motor4, AP_MOTORS_MOT_4);
    _motor_servo[AP_MOTORS_MOT_5] = SRV_Channels::get_channel_for(SRV_Channel::k_motor5, AP_MOTORS_MOT_5);
    _motor_servo[AP_MOTORS_MOT_6] = SRV_Channels::get_channel_for(SRV_Channel::k_motor6, AP_MOTORS_MOT_6);

    // find the yaw servo
    _yaw_servo1 = SRV_Channels::get_channel_for(SRV_Channel::k_motor7, AP_MOTORS_YAW_1);
    _yaw_servo2 = SRV_Channels::get_channel_for(SRV_Channel::k_motor8, AP_MOTORS_YAW_2);
    if (!_yaw_servo1 || !_yaw_servo2) {
        gcs().send_text(MAV_SEVERITY_ERROR, "MotorsTri: unable to setup yaw channel");
        // don't set initialised_ok
        return;
    }

    // allow mapping for yaw
    add_motor_num(AP_MOTORS_YAW_1);
    add_motor_num(AP_MOTORS_YAW_2);

    ConfigureMotorFactors(AP_MOTORS_MOT_1, 0);
    ConfigureMotorFactors(AP_MOTORS_MOT_2, 180);
    ConfigureMotorFactors(AP_MOTORS_MOT_3, -120);
    ConfigureMotorFactors(AP_MOTORS_MOT_4, 60);
    ConfigureMotorFactors(AP_MOTORS_MOT_5, -60);
    ConfigureMotorFactors(AP_MOTORS_MOT_6, 120);

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TILTHEXA);
}

void AP_MotorsTiltHexa::ConfigureMotorFactors(int MotorNumber, float angle_degrees)
{
	_roll_factor[MotorNumber] = cosf(radians(angle_degrees + 90));
	_pitch_factor[MotorNumber] = cosf(radians(angle_degrees));
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsTiltHexa::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TILTHEXA);
}

// set update rate to motors - a value in hertz
void AP_MotorsTiltHexa::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 6 motors (but not the yaw servos)
    uint32_t mask =
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2 |
	    1U << AP_MOTORS_MOT_3 |
    	1U << AP_MOTORS_MOT_4 |
	    1U << AP_MOTORS_MOT_5 |
		1U << AP_MOTORS_MOT_6;
    rc_set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsTiltHexa::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);
    rc_enable_ch(AP_MOTORS_MOT_2);
    rc_enable_ch(AP_MOTORS_MOT_3);
    rc_enable_ch(AP_MOTORS_MOT_4);
    rc_enable_ch(AP_MOTORS_MOT_5);
    rc_enable_ch(AP_MOTORS_MOT_6);
    rc_enable_ch(AP_MOTORS_YAW_1);
    rc_enable_ch(AP_MOTORS_YAW_2);
}

void AP_MotorsTiltHexa::output_to_motor(int MotorIndex)
{
	rc_write(MotorIndex,
		_motor_servo[MotorIndex]->get_trim() - 1500 +
			calc_thrust_to_pwm(ReverseThrustIfRequired(_thrust[MotorIndex], _motor_servo[MotorIndex])));
}

void AP_MotorsTiltHexa::output_to_motors()
{
    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_2, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_3, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_4, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_5, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_6, get_pwm_output_min());
            rc_write(AP_MOTORS_YAW_1, _yaw_servo1->get_trim());
            rc_write(AP_MOTORS_YAW_2, _yaw_servo2->get_trim());
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            rc_write(AP_MOTORS_MOT_1, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_2, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_3, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_4, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_5, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_6, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_YAW_1, _yaw_servo1->get_trim());
            rc_write(AP_MOTORS_YAW_2, _yaw_servo2->get_trim());
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
        	output_to_motor(AP_MOTORS_MOT_1);
        	output_to_motor(AP_MOTORS_MOT_2);
        	output_to_motor(AP_MOTORS_MOT_3);
        	output_to_motor(AP_MOTORS_MOT_4);
        	output_to_motor(AP_MOTORS_MOT_5);
        	output_to_motor(AP_MOTORS_MOT_6);
            rc_write(AP_MOTORS_YAW_1, calc_yaw_radio_output(_pivot_angle, radians(_yaw_servo_angle_max_deg), _yaw_servo1));
            rc_write(AP_MOTORS_YAW_2, calc_yaw_radio_output(_pivot_angle, radians(_yaw_servo_angle_max_deg), _yaw_servo2));
            //printf("_pivot_angle = %f\n", _pivot_angle);
            break;
    }
}

float AP_MotorsTiltHexa::ReverseThrustIfRequired(float Thrust, SRV_Channel *pServo)
{
	if (pServo->get_reversed())
	{
		return (1 - Thrust);
	}
	else
	{
		return Thrust;
	}
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTiltHexa::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    return rc_map_mask((1U << AP_MOTORS_MOT_1) |
                       (1U << AP_MOTORS_MOT_2) |
                       (1U << AP_MOTORS_MOT_3) |
                       (1U << AP_MOTORS_MOT_4) |
                       (1U << AP_MOTORS_MOT_5) |
                       (1U << AP_MOTORS_MOT_6) |
                       (1U << AP_MOTORS_YAW_1) |
                       (1U << AP_MOTORS_YAW_2));
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
/*
 * Calculates the required throttle position for each motor, given the required
 * roll, pitch, yaw and thrust.
 */
void AP_MotorsTiltHexa::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0

    // sanity check YAW_SV_ANGLE parameter value to avoid divide by zero
    //_yaw_servo_angle_max_deg = constrain_float(_yaw_servo_angle_max_deg, AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN, AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX);

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain()*sinf(radians(_yaw_servo_angle_max_deg)); // we scale this so a thrust request of 1.0f will ask for full servo deflection at full throttle
    //User expects to have to compensate with more thrust when there is pitch or roll.
    throttle_thrust = get_throttle() * get_compensation_gain();

    // calculate angle of yaw pivot
    _pivot_angle = safe_asin(yaw_thrust);

    {
    	int n;
    	for (n = 0; n < TILT_HEXA_QTY_MOTORS; n++)
    	{
    		_thrust[n] = constrain_float(
    				pitch_thrust * _pitch_factor[n] + roll_thrust * _roll_factor[n] + throttle_thrust,
					0, 1);
    	}
    }
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTiltHexa::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // front motor
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // front right motor
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 3:
            // back right motor
            rc_write(AP_MOTORS_MOT_6, pwm);
            break;
        case 4:
            // back motor
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 5:
			// back left motor
			rc_write(AP_MOTORS_MOT_3, pwm);
			break;
        case 6:
			// front left motor
			rc_write(AP_MOTORS_MOT_5, pwm);
			break;
        case 7:
			// tilt 1
			rc_write(AP_MOTORS_YAW_1, pwm);
			break;
        case 8:
			// tilt 2
			rc_write(AP_MOTORS_YAW_2, pwm);
			break;
        default:
            // do nothing
            break;
    }
}

// calc_yaw_radio_output - calculate final radio output for yaw channel
int16_t AP_MotorsTiltHexa::calc_yaw_radio_output(float yaw_input, float yaw_input_max, SRV_Channel *pServo)
{
    int16_t ret;

    if (pServo->get_reversed()) {
        yaw_input = -yaw_input;
    }

    if (yaw_input >= 0){
        ret = (pServo->get_trim() + (yaw_input/yaw_input_max * (pServo->get_output_max() - pServo->get_trim())));
    } else {
        ret = (pServo->get_trim() + (yaw_input/yaw_input_max * (pServo->get_trim() - pServo->get_output_min())));
    }

    return ret;
}
