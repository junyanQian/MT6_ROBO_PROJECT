#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <chprintf.h>


#include <pi_regulator.h>
#include <process_image.h>

static uint8_t main_status = STATUS_IDLE;

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void set_status(void){
	if(main_status < STATUS_ARRIV) main_status++ ;
	else main_status = STATUS_IDLE;
}

void set_status_idle(void){
	main_status = STATUS_IDLE;
}

uint8_t get_status(void){
	return main_status;
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//time of flight starts
	VL53L0X_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	if(main_status == STATUS_IDLE) set_status();
    	//chprintf((BaseSequentialStream *)&SDU1, "distance_obj = %f\n", VL53L0X_get_dist_mm());
    	chprintf((BaseSequentialStream *)&SDU1, "main_status = %d\n", main_status);
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
