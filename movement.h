/******************************************************************************
*
*       File: movement.h
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Declarations for movement module.
*
*
*******************************************************************************/


#ifndef MOVEMENT_H
#define MOVEMENT_H

/* Must define the global latency for writing to PWM in ticks. This is the
   time inbetween successive writes to the PWM module. 
   */
#include "pwm.h"
#include "messaging.h"
#include "ECD.h"
#include "sigmoid.h"


/* Config */

/* Latencies are in ticks of the system timer */
#define MOVE_LATENCY 10


#define SERVO_COUNT PWM_COUNT

/* For continuous movement */
#define MOVE_JUMPVAL 10
#define MOVE_DELTA 0.05
#define MOVE_JUMP_MAX 300

/* For specific movement */
#define MOVE_SPEC_STD_SPEED 12500      //the time allowed for all movements - random at 50000pwmvalues/4s
#define MOVE_IK_MSG_COMPRESSION 6.11   // the value that the speed gets divided by in the ik module so that it fits in the 32bit messageData
										// It gets multiplied by it in the movement module again.





/* Task priority */
#define MOVE_PRIORITY tskIDLE_PRIORITY 
#define SERVO_PRIORITY ( configMAX_PRIORITIES - 1 )

enum {
	MOVE_STATE_STOP,
	MOVE_STATE_INC,
	MOVE_STATE_DEC,
	MOVE_STATE_COUNT,
    MOVE_STATE_IK
};


/* Set up the module, give it the queue to receive messages from */ 
int move_Start(xQueueHandle);

int move_get_state(int servo, int* state);

#endif

