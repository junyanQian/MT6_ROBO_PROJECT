#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <show_status.h>

//global static value definition
static uint8_t led[NB_OF_BODYLEDS] = 	{	0, 0, 0, 0,
											0, 0, 0, 0	};
static uint8_t main_status = STATUS_IDLE;
static uint8_t led_arr_toggle = 0;

static const uint8_t LED_EVEN = 		{	0, 1, 0, 1,
											0, 1, 0, 1	};
static const uint8_t LED_ODD = 			{	1, 0, 1, 0,
											1, 0, 1, 0	};
static const uint8_t LED_TRANS_INI = 	{	1, 0, 0, 0,
											0, 0, 0, 0	};
static const uint8_t LED_ERROR = 		{	1, 1, 1, 1,
											1, 1, 1, 1	};

void show_status(void){
	if (main_status == STATUS_IDLE){
		//leds 2 4 6 8 turn on
		led = LED_EVEN;
	}else if (main_status == STATUS_SEARCH){
		//all leds blink, periode = 1s
		for (int i = 0; i < NB_OF_BODYLEDS; i++){
			led[i] = led[i] ? 0 : 1;
		}
	}else if (main_status == STATUS_FETCH){
		//led1 blink
		led[0] = led[0] ? 0 : 1;
		for (int i = 1; i < NB_OF_BODYLEDS; i++){
			led[i] = 0;
		}
	}else if (main_status == STATUS_TRANS){
		//leds rotate, clockwise, periode =1s
		//initiated in main when changing the status to trans
		for (int i = 0; i < NB_OF_BODYLEDS; i++){
			if (led[i] == 1 && i < NB_OF_BODYLEDS-1){
				led[i] = 0;
				led[i+1] = 1;
				break;
			}else if (led[i] == 1 && i == NB_OF_BODYLEDS-1){
				led[i] = 0;
				led[0] = 1;
				break;
			}
		}
	}else if (main_status == STATUS_ARRIV){
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

static THD_WORKING_AREA(waShowStatus, 256);

static THD_FUNCTION(ShowStatus, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        show_status();

        //8Hz
        chThdSleepUntilWindowed(time, time + MS2ST(125));
    }
}
void show_status_start(void){
	chThdCreateStatic(waShowStatus, sizeof(waShowStatus), NORMALPRIO, ShowStatus, NULL);
}
