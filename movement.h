#ifndef MOVEMENT_H
#define MOVEMENT_H

/* Must define the global latency for writing to PWM in ticks. This is the
   time inbetween successive writes to the PWM module. 
   */
#include "pwm.h"
#include "messaging.h"
#include "ECD.h"

/* Config */

/* Latencies are in ticks of the system timer */
#define MOVE_LATENCY 50

/* ms between steps  - too small isn't going to get scheduled */
#define MOVE_SIGMOID_LATENCY MOVE_LATENCY


#define SERVO_COUNT PWM_COUNT

/* For continuous movement */
#define MOVE_JUMPVAL 200
#define MOVE_DELTA 1
#define MOVE_JUMP_MAX 1500

/* For specific movement */
#define MOVE_SPEC_STD_SPEED 4      //the time allowed for all movements - random atm

#define SIGMOID_ERR	6.2126




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

