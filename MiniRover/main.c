#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>

//include sensors
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>

//include processing algorithms
#include <process_image.h>
#include <navigation.h>

//initialize a static variable to control the flow of the program
static uint8_t main_status = STATUS_IDLE;

//declare functions to manipulate the static variable
void set_main_status(void){
	if (main_status < STATUS_ARRIV) main_status++;
}
uint8_t get_main_status(void){
	return main_status;
}

//functions related to communication
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

int main(void)
{
	//main processor initialization
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
	VL53L0X_start();
	//inits the motors
	motors_init();
	//inits working threads
	chThdSetPriority(NORMALPRIO+3);
	navigation_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
    	//initiate the program
    	if(main_status == STATUS_IDLE) set_main_status();
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
