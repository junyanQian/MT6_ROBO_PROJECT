#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			1
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define SPEED_ROTATION 			50
#define SPEED_FORWARD 			400
#define MARGIN					1.5
#define LIMIT_TOF				60
#define STEP_FETCH				200
#define VERIFIED				2
//definition of states
#define STATUS_IDLE				0
#define STATUS_SEARCH			1
#define STATUS_FETCH			2
#define STATUS_TRANS			3
#define STATUS_ARRIV			4

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);
void set_main_status(void);
uint8_t get_main_status(void);

#ifdef __cplusplus
}
#endif

#endif
