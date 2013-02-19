/*
 *
 *
 *
 *    Code for Servo Movement interaction
 *
 */

#include "movement.h"
#include "math.h"
#include <system.h>
#include <stdlib.h>

/* This is the INCOMING queue */
static xQueueHandle qMove;

static xSemaphoreHandle xSemaphore = NULL;

/* Position info is kept in the servo task */
typedef struct {
	xQueueHandle qServo;
	unsigned int iServoID;
	unsigned position;
	int state;
} move_servoData_s;

/* Struct for euler stuff */
typedef struct {
	float input;
	float output;
	unsigned int state;
	unsigned int test;
} euler_s;

static int sigmoid(float M, float time, float*result); /* Find sigmoid position */
static void move_servo_sigmoid(move_servoData_s *sData, int place, int speed); /* Move to a specified place, in a specified time */
static void move_servo_task(void *params); /* Individual servo tasks, one spawned for each servo */
static void move_main_task(void* params); /* Main task manager for this module */
static void move_servo_cont(move_servoData_s *sData, int direction); /* Move loop */

/* These are the queues going off to the servo tasks */
static move_servoData_s ServoData[SERVO_COUNT];

int move_Start(xQueueHandle qHandle) {

	int x;

	/* Sanity */
	if (qHandle == NULL)
		return ECD_ERROR;

	/* Keep copy of the Queue Handle to use */
	qMove = qHandle;

	/* Create all the servo tasks and their queues */
	for (x = 0; x < SERVO_COUNT; x++) {

		/* Create queue */
		if (msg_newQueue(&ServoData[x].qServo) != ECD_OK)
			return ECD_ERROR;

		ServoData[x].iServoID = x;

		/* Create task, pass queue handle in */
		if (xTaskCreate( move_servo_task,
				(fStr)"SERVOTASK",
				configMINIMAL_STACK_SIZE,
				(void*)&ServoData[x],
				SERVO_PRIORITY,
				NULL) != pdPASS) {

			return ECD_ERROR;
		}

	}

	xSemaphore = xSemaphoreCreateMutex();

	/* Create main task; return -1 on error */
	if (xTaskCreate( move_main_task,
			(fStr)"Movement Main Thread",
			configMINIMAL_STACK_SIZE,
			NULL,
			MOVE_PRIORITY,
			NULL) != pdPASS) {

		return ECD_ERROR;
	}

	return ECD_OK;
}

static void move_main_task(void* params) {

	msg_message_s msgMessage;
	int servoID;

	printf("Movement main task created...\n");

	for (;;) {
		/* So now wait for commands to come through from the Main Manager
		 and send them to the relevant servo task to execute */
		msg_recv_block(qMove, &msgMessage);

		switch (msgMessage.messageID) {

		case M_MOVE_CONT:
		case M_MOVE_STOP:
		case M_MOVE_SPEC:

			/* Mask off 8bit servo number */
			servoID = M_MOVE_SERVOMASK & msgMessage.messageDATA;

			/* If bad servo id quit */
			if (servoID >= PWM_COUNT) {
				printf("Bad Servo ID! \n");
				break;
			}

			/* Send out message */
			msg_send(ServoData[servoID].qServo, msgMessage);

			break;
		case M_MOVE_IK:
			/* Mask off 4bit servo number */
			servoID = ((msgMessage.messageDATA & M_MOVE_SERVOMASK_IK) >> 1);
			/* If bad servo id quit */
			if (servoID >= PWM_COUNT) {
				printf("Bad Servo ID! - IK \n");
				break;
			}
			/* Send out message */
			msg_send(ServoData[servoID].qServo, msgMessage);

			break;
		default:
			break;
		}

	}

	return;
}

/* This function is served to each of the servo tasks */
static void move_servo_task(void *params) {

	move_servoData_s servoData;
	msg_message_s msgMessage;

	/* Grab local copy of the queue handle */
	servoData = *(move_servoData_s*) params;

	//printf("I am servo task %d.\n", servoData.iServoID);

	for (;;) {

		/* Block on queue until message received */
		msg_recv_block(servoData.qServo, &msgMessage);

		/* Is it a Specific Move, or Continous Move command? */
		switch (msgMessage.messageID) {
		case M_MOVE_CONT:
			/* Received command to start continous movement - do so until a stop message is received */
			move_servo_cont(&servoData,
					(msgMessage.messageDATA & M_MOVE_DIRMASK));

			/* As move_servo_cont is returned, we must have stopped */
			ServoData[servoData.iServoID].state = MOVE_STATE_STOP;

			break;

			/* Move to a specific place using Sigmoid function */
		case M_MOVE_IK:
			ServoData[servoData.iServoID].state = MOVE_STATE_IK;

			printf("Servo %d PWM value: %d \n", servoData.iServoID,
					(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)
							>> M_MOVE_PWMOFFSET_IK) + 50000));

			if ((msgMessage.messageDATA & M_MOVE_SPECSPEEDMASK_IK)
					>> M_MOVE_SPECSPEEDOFFSET_IK) {

				/*if a movement speed is defined */

				//move_servo_sigmoid(&servoData,(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000), ((msgMessage.messageDATA & M_MOVE_SPECSPEEDMASK_IK)>>M_MOVE_SPECSPEEDOFFSET_IK));
				/*move_servo_sigmoid(
				 &servoData,
				 (((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000),
				 MOVE_SPEC_STD_SPEED //M_MOVE_SPEC_SPEED(msgMessage.messageDATA)
				 );*/

				pwm_set_pos(servoData.iServoID,
						(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)
								>> M_MOVE_PWMOFFSET_IK) + 50000));

			} else {

				//move_servo_sigmoid(&servoData,(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000), MOVE_SPEC_STD_SPEED);
				/*move_servo_sigmoid(
				 &servoData,
				 (((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000),
				 MOVE_SPEC_STD_SPEED //M_MOVE_SPEC_SPEED(msgMessage.messageDATA)
				 );*/
				pwm_set_pos(servoData.iServoID,
						(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)
								>> M_MOVE_PWMOFFSET_IK) + 50000));

			}
			ServoData[servoData.iServoID].state = MOVE_STATE_STOP;
			break;

		case M_MOVE_SPEC:
			/* Servo, place ,speed */
			/* Currently the position values for the PWM are between 50,000 and 100,000 -
			 * to use two halves of the messgae DAta int, we could just use 0 to 50,000 - but that
			 * might be confusing... */
			move_servo_sigmoid(&servoData,
					M_MOVE_SPEC_POSITION(msgMessage.messageDATA),
					M_MOVE_SPEC_SPEED(msgMessage.messageDATA));

			break;

		case M_MOVE_STOP:
			/* This means the motor is stopped, but we've received a pointless STOP request */
			printf("Servo task %d received unneccessary stop message.\n",
					servoData.iServoID);
			//printf("Stopping movement on servo %d.\n",msgMessage.messageDATA);
			break;
		default:
			break;
		}

	}
}

/* Called from within the servo tasks to initiate a continous movement in a specified direction
 * but with no end-point until a STOP message is received. It's useful for when moving the arm
 * around using the keypad in the free movement mode.*/
static void move_servo_cont(move_servoData_s *sData, int direction) {

	msg_message_s msgMessage;
	int jumpval = MOVE_JUMPVAL;
	int delta = 0;

	if (direction & M_MOVE_DIRMASK)
		ServoData[sData->iServoID].state = MOVE_STATE_INC;
	else
		ServoData[sData->iServoID].state = MOVE_STATE_DEC;

	for (;;) {

		/* Quick message check  */
		if (msg_recv_noblock(sData->qServo, &msgMessage) != ECD_NOMSG) {
			if (msgMessage.messageID != M_MOVE_STOP) {
				printf("Expecting STOP message but received something else!\n");
				return;
			} else {
				//printf("Servo task %d STOPPING %s.\n", sData->iServoID,direction ? "INC": "DEC");
				return;
			}
		}

		/* So no message received, move one step */
		//printf("Servo task %d moving %s STEP.\n", sData->iServoID,direction ? "INC": "DEC");
		if (direction & M_MOVE_DIRMASK)
			pwm_jump(sData->iServoID, jumpval);
		else
			pwm_jump(sData->iServoID, -jumpval);

		/* Increase movement speed until MOVE_JUMP_MAX */
		if (jumpval < (MOVE_JUMP_MAX)) {
			jumpval += delta;
			delta += MOVE_DELTA;
		}

		/* If it goes over just set it to max */
		if (jumpval > MOVE_JUMP_MAX)
			jumpval = MOVE_JUMP_MAX;

		vTaskDelay(MOVE_LATENCY);

	}

}

int move_get_state(int servo, int*state) {

	/* This might need mutex protection if it become an important thing, atm its
	 just used in displaying so not really worth it. */
	*state = ServoData[servo].state;

	return ECD_OK;
}

/* A normalized sigmoid - result is always 0-1
 * M is a time value which must be the same unit as the "time" parameter */
static int sigmoid(float M, float time, float*result) {

	euler_s* p_euler;
	float fInput;

	/* The M value is the half time point where the gradient is maximum */

	/* Boundary checks ? */

	/* Do the sigmoid calcs */
	/* TODO: Replace this with IORD/IOWR macros */
	p_euler = (euler_s*) EULERBLOCK_0_BASE;

	fInput = (-1 * (SIGMOID_ERR) * (time - M)) / M;

	if (xSemaphore != NULL) {
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE) {
			/* Wait for the euler block to become ready */
			while (!p_euler->state)
				vTaskDelay(1);/*definitely 0, remove if redundant?*/

			p_euler->input = fInput;

			/* Wait for the euler block to become ready */
			while (!p_euler->state)
				vTaskDelay(1);

			xSemaphoreGive( xSemaphore);
		} else {
			printf("Servo couldn't get semaphore! Why ever not?\n");
			// We could not obtain the semaphore and can therefore not access
			// the shared resource safely.
		}
	}

	*result = p_euler->output;

	return ECD_OK;
}

/* This function completes a full movement from where the PWM currently is
 * to where it is specified to go. At that point it returns. During this movement,
 * a STOP message can be sent which will force an early return */
static void move_servo_sigmoid(move_servoData_s *sData, int place, int speed) {

	msg_message_s msgMessage;
	unsigned int initialposition;
	signed int distance;
	float m, totaltime;
	int n = 0;
	float res = 0;
	float latency_ms = 0;

	/* First work out how far we have to travel */
	pwm_get_pos(sData->iServoID, &initialposition);
	distance = (signed int) initialposition - (signed int) place;

	/* We've got no-where to go */
	if (abs(distance) < 10)
		return;

	/* This distance value will act as the scale factor for the Sigmoid function
	 * it's signed becuase we can of course be going in two directions */

	/* The speed for the transition is passed in so find the total time
	 * but it can't be negative. */
	totaltime = fabs((float) distance) / (float) speed;

	/* M is the half-way point which is passed to the sigmoid function */
	m = 1000 * totaltime / 2.0;

	latency_ms = TICKS2MS(MOVE_LATENCY);

	/* So start main loop */
	for (;;) {

		/* Quick message check */
		if (msg_recv_noblock(sData->qServo, &msgMessage) != ECD_NOMSG) {
			if (msgMessage.messageID != M_MOVE_STOP) {
				printf(
						"Expecting STOP message but received something else! Returning!\n");
				return;
			} else {
				printf("Servo MID-MOVE but received non-STOP message!\n");
				return;
			}
		}

		/* So no STOP message received, move one step */

		/* Get normalized value */
		sigmoid(m, latency_ms, &res);

		/* Now scale it */
		res *= distance;

		/* And add the initial offset */
		res += initialposition;

		/* Are we there yet? */
		if ((abs(place) - abs(res)) > 100) {

			/* Now move there */
			pwm_set_pos(sData->iServoID, (unsigned int) res);

			//n++;
			latency_ms += latency_ms;
		}
		else {
			return;
		}

		vTaskDelay(MOVE_LATENCY);

	}

}

