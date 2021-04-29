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
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

//global static value definition
static uint8_t led[NB_OF_BODYLEDS] = {	0, 0, 0, 0,
										0, 0, 0, 0	};
static uint8_t status = STATUS_IDLE;
static uint8_t led_arr_toggle = 0;

static const uint8_t LED_EVEN = {	0, 1, 0, 1,
									0, 1, 0, 1	};
static const uint8_t LED_ODD = {	1, 0, 1, 0,
									1, 0, 1, 0	};
static const uint8_t LED_TRANS_INI = {	1, 0, 0, 0,
										0, 0, 0, 0	};
static const uint8_t LED_ERROR = {	1, 1, 1, 1,
									1, 1, 1, 1	};

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
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {


    	show_status(status);
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

void show_status(void){
	if (status == STATUS_IDLE){
		//leds 2 4 6 8 turn on
		led = LED_EVEN;
	}else if (status == STATUS_SEARCH){
		//all leds blink, periode = 1s
		for (int i = 0; i < NB_OF_BODYLEDS; i++){
			led[i] = led[i] ? 0 : 1;
		}
	}else if (status == STATUS_FETCH){
		//led1 blink
		led[0] = led[0] ? 0 : 1;
		for (int i = 1; i < NB_OF_BODYLEDS; i++){
			led[i] = 0;
		}
	}else if (status == STATUS_TRANS){
		//leds rotate, clockwise, periode = 1s
		//initiated in main when changing the status to trans
		for (int i = 0; i < NB_OF_BODYLEDS; i++){
			if (led[i] == 1 && i != NB_OF_BODYLEDS-1){
				led[i] = 0;
				led[i+1] = 1;
				break;
			}else if (led[i] == 1 && i == NB_OF_BODYLEDS-1){
				led[i] = 0;
				led[0] = 1;
				break;
			}
		}
	}else if (status == STATUS_ARRIV){
		//even leds and odd led invert
		if(led_arr_toggle){
			led = LED_EVEN;
			led_arr_toggle = 0;
		}
		else {
			led = LED_ODD;
			led_arr_toggle = 1;
		}
	}else {
		//all leds turned on
		led = LED_ERROR;
	}
	palWritePad(GPIOD, GPIOD_LED1, led[0] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED3, led[1] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED5, led[2] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED7, led[3] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED1, led[4] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED3, led[5] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED5, led[6] ? 0 : 1);
	palWritePad(GPIOD, GPIOD_LED7, led[7] ? 0 : 1);

}
