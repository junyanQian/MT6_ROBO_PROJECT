#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <pal.h>
#include <pi_regulator.h>
#include <process_image.h>




//simple PI regulator implementation
/*int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}
*/

bool search_target(void){
	if((get_line_position() >= 280) && (get_line_position() <= 360)){
		return true;
	}else return false;
}

void approach_object(void){
	int16_t speed_correction = 0;
	speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
	if(abs(speed_correction) < ROTATION_THRESHOLD) speed_correction = 0;
	//zig-zaging towards the target
	right_motor_set_speed(SPEED_SEARCH-(ROTATION_COEFF*speed_correction));
	left_motor_set_speed(SPEED_SEARCH+(ROTATION_COEFF*speed_correction));
}

void stop_moving(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void set_step_zero(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void fetch_object(void){
	right_motor_set_speed(SPEED_FETCH);
	left_motor_set_speed(SPEED_FETCH);
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //int16_t speed = 0;
    float distance_obj = 0;
    bool target_found = 0;

    while(1)
    {
        time = chVTGetSystemTime();
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        //speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
        //computes a correction factor to let the robot rotate to be in front of the line
        distance_obj = VL53L0X_get_dist_mm();
        //chprintf((BaseSequentialStream *)&SDU1, "distance_obj = %f\n", distance_obj);
        if(get_status() == STATUS_SEARCH){
        	target_found = search_target();
        	if(!target_found){
        		right_motor_set_speed(SPEED_ROTATION);
        		left_motor_set_speed(-SPEED_ROTATION);
        	}else{
        		approach_object();
        		if(distance_obj < LIMIT_TOF){
        			stop_moving();
        		    set_status();
        		    set_step_zero();
        		}
        	}
        }
        if(get_status() == STATUS_FETCH){
        	if(left_motor_get_pos() < STEP_FETCH){
        		fetch_object();
        	}else{
        		stop_moving();
        		set_status_idle();
        	}
        }

       // right_motor_set_speed(avancer_speed);
       // left_motor_set_speed(avancer_speed);

        if(target_found)
        	palWritePad(GPIOD, GPIOD_LED7, 0);
        else
        	palWritePad(GPIOD, GPIOD_LED7, 1);

        if(distance_obj < LIMIT_TOF)
        	palWritePad(GPIOD, GPIOD_LED5, 0);
        else
        	palWritePad(GPIOD, GPIOD_LED5, 1);

			//chprintf((BaseSequentialStream *)&SDU1, "distance = %f\n", distance_obj);

			//applies the speed from the PI regulator and the correction for the rotation
			//right_motor_set_speed(avancer_speed - ROTATION_COEFF * speed_correction);
			//left_motor_set_speed(avancer_speed + ROTATION_COEFF * speed_correction);

			//100Hz
			chThdSleepUntilWindowed(time, time + MS2ST(100));

    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
