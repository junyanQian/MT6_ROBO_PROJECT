#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <navigation.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <msgbus/messagebus.h>
#include <sensors/imu.h>

//communication with IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
//declare a verifier to avoid accidental motor_stop from fluctuation
static uint8_t verifier = 0;
//motor functions
static void turn_left(uint16_t speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);
}
static void turn_right(uint16_t speed){
	right_motor_set_speed(-speed);
	left_motor_set_speed(speed);
}
static void go_straight(void){
	right_motor_set_speed(SPEED_FORWARD);
	left_motor_set_speed(SPEED_FORWARD);
}
static void stop_motor(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}
static void set_motorPos_zero(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
//program regulation functions
static bool search_target(void){
	if((get_line_position() >= 180) && (get_line_position() <= 460)) return true;
	else return false;
}
static void approach_object(void){
	int16_t speed_correction = 0;
	speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
	if(abs(speed_correction) < ROTATION_THRESHOLD) speed_correction = 0;
	//zig-zaging towards the target
	right_motor_set_speed(SPEED_FORWARD-(ROTATION_COEFF*speed_correction));
	left_motor_set_speed(SPEED_FORWARD+(ROTATION_COEFF*speed_correction));
}
static void set_verifier(void){
	verifier++;
}
static void zero_verifier(void){
	verifier = 0;
}
static void transportation(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    //uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 0.3;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;

    /*
    *        	 FRONT
    * 			 ##led1
    * 		   #  -Y   #
    *	   # #           # #
    * 	   ##             ##
    *  led7## +X       -X ##led3
    *      # #           # #
    *          #  +Y   #
    *            ##led5
    *            BACK
    */
    /*
    if (fabs( accel[Y_AXIS]) > fabs(accel[X_AXIS])){
    	if(accel[Y_AXIS] < -threshold)led5 = 1;
    	else if(accel[Y_AXIS] > threshold) led1 = 1;
    }
    if (fabs( accel[Y_AXIS]) < fabs(accel[X_AXIS])){
    	if(accel[X_AXIS] < -threshold) led7 = 1;
        else if(accel[X_AXIS] > threshold) led3 = 1;
    }
    */
    if (accel[Y_AXIS] < -threshold){
    	if(accel[X_AXIS] < -threshold){
    		turn_left(SPEED_ROTATION);
    		zero_verifier();
    	}
    	else {
    		turn_right(SPEED_ROTATION);
    		zero_verifier();
    	}
    }else if(accel[Y_AXIS] >= -threshold){
    	if(accel[X_AXIS] < -MARGIN*threshold){
    		turn_left(SPEED_ROTATION);
    		zero_verifier();
    	}
    	else if(accel[X_AXIS] > MARGIN*threshold){
    		turn_right(SPEED_ROTATION);
    		zero_verifier();
    	}
    	else if(accel[Y_AXIS] > threshold){
    		go_straight();
    		chThdSleepMilliseconds(50);
    		zero_verifier();
    	}
    	else set_verifier();
    }
    if(verifier == VERIFIED){
    	stop_motor();
    	zero_verifier();
    	set_main_status();
    }
    //to see the duration on the console
    /*
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);
    */
}

//***********************************  Declaration of the thread ********************************

static THD_WORKING_AREA(waNavigation, 256);
static THD_FUNCTION(Navigation, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    imu_start();
    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //local variable declaration and initiation
    systime_t time;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;
    bool target_found = false;
    uint16_t object_distance = 0;
    uint8_t nav_status = 0;
    while(1){
        time = chVTGetSystemTime();
        //aquire data from other threads
        nav_status = get_main_status();
        object_distance = VL53L0X_get_dist_mm();

        //**********************************     SEARCH    ***************************************
        if(nav_status == STATUS_SEARCH){
        	chBSemWait(&image_process_ready_sem);
        	target_found = search_target();
        	palWritePad(GPIOD, GPIOD_LED1, 0);
			if(!target_found) turn_left(SPEED_ROTATION);
			else{
				approach_object();
				if(object_distance < LIMIT_TOF){
					stop_motor();
					set_motorPos_zero();
					set_main_status();
				}
			}
        }else palWritePad(GPIOD, GPIOD_LED1, 1);

        //**********************************     Fetch     ***************************************
        if(nav_status == STATUS_FETCH){
        	palWritePad(GPIOD, GPIOD_LED3, 0);
        	if (left_motor_get_pos() < STEP_FETCH){
        		go_straight();
        	}else{
        		stop_motor();
        		set_main_status();
        		//imu_start();
        	}
        }else palWritePad(GPIOD, GPIOD_LED3, 1);

        //**********************************   TRANSPORT   ***************************************
        if(nav_status == STATUS_TRANS){
        	palWritePad(GPIOD, GPIOD_LED5, 0);
        	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
        	transportation(&imu_values);
        }else palWritePad(GPIOD, GPIOD_LED5, 1);

        //**********************************    ARRIVED    ***************************************
        if(nav_status == STATUS_ARRIV){
        	palWritePad(GPIOD, GPIOD_LED7, 0);
        }else palWritePad(GPIOD, GPIOD_LED7, 1);

        //10Hz
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

void navigation_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+2, Navigation, NULL);
}
