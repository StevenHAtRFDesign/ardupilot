/*
 * AP_MotorsTiltHexa.h
 *
 *  Created on: 13Sep.,2017
 *      Author: snh
 */

#ifndef LIBRARIES_AP_MOTORS_AP_MOTORSTILTHEXA_H_
#define LIBRARIES_AP_MOTORS_AP_MOTORSTILTHEXA_H_

#include "AP_Motors_Class.h"

// tail servo uses channel 7
#define AP_MOTORS_YAW_1    AP_MOTORS_MOT_7
#define AP_MOTORS_YAW_2    AP_MOTORS_MOT_8

class AP_MotorsTiltHexa : public AP_MotorsMulticopter {
public:
	/// Constructor
	AP_MotorsTiltHexa(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
	    AP_MotorsMulticopter(loop_rate, speed_hz)
	{
	};

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type);

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type);

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask();

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    //void                output_motor_mask(float thrust, uint8_t mask) override;

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();

    // call vehicle supplied thrust compensation if set
    //void                thrust_compensation(void) override;

    // calc_yaw_radio_output - calculate final radio output for yaw channel
    int16_t             calc_yaw_radio_output(float yaw_input, float yaw_input_max, SRV_Channel *pServo);        // calculate radio output for yaw servo, typically in range of 1100-1900

    // parameters

    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    //float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)

    SRV_Channel     *_yaw_servo1; // yaw output channel
    SRV_Channel     *_yaw_servo2; // yaw output channel
    float           _pivot_angle;                       // Angle of yaw pivot
    /*
     * 1 - front
     * 2 - rear
     * 3 - rear left
     * 4 - front right
     * 5 - front left
     * 6 - rear right
     */
    float			_thrust[AP_MOTORS_MAX_NUM_MOTORS];

private:
    void ConfigureMotorFactors(int MotorNumber, float angle_degrees);
    int16_t             InvertPWM(int16_t PWM);
};



#endif /* LIBRARIES_AP_MOTORS_AP_MOTORSTILTHEXA_H_ */
