/*
 * AP_MotorsTiltHexa.cpp
 *
 *  Created on: 13Sep.,2017
 *      Author: snh
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsTiltHexa.h"

#define TILT_HEXA_QTY_MOTORS 6

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsTiltHexa::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
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
    _motor_servo[AP_MOTORS_MOT_2] = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, AP_MOTORS_MOT_1);
    _motor_servo[AP_MOTORS_MOT_3] = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, AP_MOTORS_MOT_1);
    _motor_servo[AP_MOTORS_MOT_4] = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, AP_MOTORS_MOT_1);
    _motor_servo[AP_MOTORS_MOT_5] = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, AP_MOTORS_MOT_1);
    _motor_servo[AP_MOTORS_MOT_6] = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, AP_MOTORS_MOT_1);

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
            rc_write(AP_MOTORS_MOT_1, ReversePWMIfRequired(calc_thrust_to_pwm(_thrust[0]), _motor_servo[AP_MOTORS_MOT_1]));
            rc_write(AP_MOTORS_MOT_2, ReversePWMIfRequired(calc_thrust_to_pwm(_thrust[1]), _motor_servo[AP_MOTORS_MOT_2]));
            rc_write(AP_MOTORS_MOT_3, ReversePWMIfRequired(calc_thrust_to_pwm(_thrust[2]), _motor_servo[AP_MOTORS_MOT_3]));
            rc_write(AP_MOTORS_MOT_4, ReversePWMIfRequired(calc_thrust_to_pwm(_thrust[3]), _motor_servo[AP_MOTORS_MOT_4]));
            rc_write(AP_MOTORS_MOT_5, ReversePWMIfRequired(calc_thrust_to_pwm(_thrust[4]), _motor_servo[AP_MOTORS_MOT_5]));
            rc_write(AP_MOTORS_MOT_6, ReversePWMIfRequired(calc_thrust_to_pwm(_thrust[5]), _motor_servo[AP_MOTORS_MOT_6]));
            rc_write(AP_MOTORS_YAW_1, calc_yaw_radio_output(_pivot_angle, radians(_yaw_servo_angle_max_deg), _yaw_servo1));
            rc_write(AP_MOTORS_YAW_2, calc_yaw_radio_output(_pivot_angle, radians(_yaw_servo_angle_max_deg), _yaw_servo2));
            break;
    }
}

int16_t AP_MotorsTiltHexa::ReversePWMIfRequired(int16_t PWM, SRV_Channel *pServo)
{
	if (pServo->get_reversed())
	{
		return (3000 - PWM);
	}
	else
	{
		return PWM;
	}
    //return get_pwm_output_min() + (get_pwm_output_max()-get_pwm_output_min()) * (_spin_min + (_spin_max-_spin_min)*apply_thrust_curve_and_volt_scaling(thrust_in));
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
    /*if (fabsf(_pivot_angle) > radians(_yaw_servo_angle_max_deg)) {
        limit.yaw = true;
        _pivot_angle = constrain_float(_pivot_angle, -radians(_yaw_servo_angle_max_deg), radians(_yaw_servo_angle_max_deg));
    }*/

    /*float pivot_thrust_max = cosf(_pivot_angle);
    float thrust_max = 1.0f;*/

    // sanity check throttle is above zero and below current limited throttle
    /*if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);*/

    {
    	int n;
    	for (n = 0; n < TILT_HEXA_QTY_MOTORS; n++)
    	{
    		_thrust[n] = constrain_float(
    				pitch_thrust * _pitch_factor[n] + roll_thrust * _roll_factor[n] + throttle_thrust,
					0, 1);
    		/*if (n == 0)
    		{
    			rpy_high = rpy_low = _thrust[n];
    		}
    		else
    		{
    			if (_thrust[n] > rpy_high)
    			{
    				rpy_high = _thrust[n];
    			}
    			if (_thrust[n] < rpy_low)
    			{
    				rpy_low = _thrust[n];
    			}
    		}*/
    	}
    }

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors

    // record lowest roll pitch command
    /*rpy_low = MIN(_thrust_right,_thrust_left);
    rpy_high = MAX(_thrust_right,_thrust_left);
    if (rpy_low > _thrust_rear){
        rpy_low = _thrust_rear;
    }*/
    // check to see if the rear motor will reach maximum thrust before the front two motors
    /*if ((1.0f - rpy_high) > (pivot_thrust_max - _thrust_rear)){
        thrust_max = pivot_thrust_max;
        rpy_high = _thrust_rear;
    }*/

    // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible room margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // check everything fits
    /*throttle_thrust_best_rpy = MIN(0.5f*thrust_max - (rpy_low+rpy_high)/2.0, _throttle_avg_max);
    if(is_zero(rpy_low)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if(rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        if (thr_adj > 0.0f){
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    }else{
        if(thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
        }else if(thr_adj > thrust_max - (throttle_thrust_best_rpy+rpy_high)){
            // Throttle can't be increased to desired value
            thr_adj = thrust_max - (throttle_thrust_best_rpy+rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    _thrust_right = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_right;
    _thrust_left = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_left;
    _thrust_rear = throttle_thrust_best_rpy+thr_adj + rpy_scale*_thrust_rear;

    // scale pivot thrust to account for pivot angle
    // we should not need to check for divide by zero as _pivot_angle is constrained to the 5deg ~ 80 deg range
    _thrust_rear = _thrust_rear/cosf(_pivot_angle);

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);*/
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
