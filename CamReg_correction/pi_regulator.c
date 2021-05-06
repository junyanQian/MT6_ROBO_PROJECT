#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pal.h>
#include <pi_regulator.h>
#include <process_image.h>




//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

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

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    int16_t avancer_speed = 0;
    int16_t avancer_speed_proximite = 150;
    float distance_obj = 0;


    while(1)
    {
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));


        if((get_line_position() > 280) && (get_line_position() < 360))
        {
        	avancer_speed = 150;
        }
        else
        {
        	avancer_speed = 0;
        }


        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD)
        	speed_correction = 0;

       // right_motor_set_speed(avancer_speed);
       // left_motor_set_speed(avancer_speed);
        distance_obj = VL53L0X_get_dist_mm();



        if(avancer_speed)
        	palWritePad(GPIOD, GPIOD_LED7, 0);
        else
        	palWritePad(GPIOD, GPIOD_LED7, 1);

        if(distance_obj < 50)
        	palWritePad(GPIOD, GPIOD_LED5, 0);
        else
        	palWritePad(GPIOD, GPIOD_LED5, 1);



        if(avancer_speed > 0)
        {
        	left_motor_set_pos(0);
        	right_motor_set_pos(0);
        	right_motor_set_speed(avancer_speed - (ROTATION_COEFF * speed_correction));
        	left_motor_set_speed(avancer_speed + (ROTATION_COEFF * speed_correction));
        	chprintf((BaseSequentialStream *)&SDU1, "distance = %d\n", left_motor_get_pos());
        }
        else if((distance_obj < 50))
        {
        	avancer_speed = 0;
        	speed_correction = 0;
        	right_motor_set_speed(avancer_speed_proximite);
        	left_motor_set_speed(avancer_speed_proximite);
        }
        else
        {
        	right_motor_set_speed(50);
        	left_motor_set_speed(-50);
        }


        //chprintf((BaseSequentialStream *)&SDU1, "distance = %f\n", distance_obj);


        //applies the speed from the PI regulator and the correction for the rotation
		//right_motor_set_speed(avancer_speed - ROTATION_COEFF * speed_correction);
		//left_motor_set_speed(avancer_speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
