
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
#include <io.h>
#include <unistd.h>

#include "pwm.h"


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
	unsigned int pos[4];
	int x = 0;
	int state[4];

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

		printf("Servo %d PWM value: %d \n", servoData.iServoID,(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>> M_MOVE_PWMOFFSET_IK) + 50000));

			if ((msgMessage.messageDATA & M_MOVE_SPECSPEEDMASK_IK)
					>> M_MOVE_SPECSPEEDOFFSET_IK) {

				/*if a movement speed is defined */
#ifndef IK_ONLY
				move_servo_sigmoid(
				 &servoData,
				 (((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000),
				 MOVE_SPEC_STD_SPEED
				 );

#if 1
		for(x=0;x<PWM_COUNT;x++){
			pwm_get_pos(x,&pos[x]);
			pos[x]/=1000;
			move_get_state(x,&state[x]);
		}



		printf("move done [%u:%u],[%u:%u],[%u:%u],[%u:%u].\n",
			pos[0],state[0],
			pos[1],state[1],
			pos[2],state[2],
			pos[3],state[3]);
		fflush( stdout );
#endif
#else
				pwm_set_pos(servoData.iServoID,
						(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)
								>> M_MOVE_PWMOFFSET_IK) + 50000));
#endif

			} else {

#ifndef IK_ONLY

				move_servo_sigmoid(
				 &servoData,
				 (((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000),
				 MOVE_SPEC_STD_SPEED //M_MOVE_SPEC_SPEED(msgMessage.messageDATA)
				 );

#if 1
		for(x=0;x<PWM_COUNT;x++){
			pwm_get_pos(x,&pos[x]);
			pos[x]/=1000;
			move_get_state(x,&state[x]);
		}



		printf("move done [%u:%u],[%u:%u],[%u:%u],[%u:%u].\n",
			pos[0],state[0],
			pos[1],state[1],
			pos[2],state[2],
			pos[3],state[3]);
		fflush( stdout );
#endif

#else
				pwm_set_pos(servoData.iServoID,
						(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)
								>> M_MOVE_PWMOFFSET_IK) + 50000));
				//move_servo_sigmoid(&servoData,(((msgMessage.messageDATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+50000), ((msgMessage.messageDATA & M_MOVE_SPECSPEEDMASK_IK)>>M_MOVE_SPECSPEEDOFFSET_IK));
#endif
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


int test_sigmoid(void){


	float M,time,result;
	volatile euler_s* p_euler;


	p_euler = (euler_s*) EULERBLOCK_0_BASE;

	/* Semaphore is normally created by the movement init function.. */
	xSemaphore = xSemaphoreCreateMutex();

	M = 2.24;
	time = 2.24;
	result = 0;

	/* Do initial write to the first place. */
	p_euler->input = M;

	/* Check it's corrent */


	printf("Starting sigmoid test....\n");
	printf("Sigmoid block test register returns [%X].\n", p_euler->test);
	printf("Floats have byte count of [%d].\n",sizeof(float));
	printf("Sigmoid returns for [%4.2f] ---> [%4.2f].\n",M,p_euler->input);

	printf("Inputs are M=[%4.2f], time=[%4.2f].\n",M,time);

	usleep(3000);

	sigmoid(M, time, &result);

	printf("Sigmoid returned [%4.2f].\n", result);

	return 0;
}




/* A normalized sigmoid - result is always 0-1
 * M is a time value which must be the same unit as the "time" parameter and represents
 * the half-time point*/
static int sigmoid(float M, float time, float*result) {

	volatile euler_s* p_euler;
	float fInput;

	/* The M value is the half time point where the gradient is maximum */

	/* Boundary checks ? */

	/* Do the sigmoid calcs */
	p_euler = (euler_s*) EULERBLOCK_0_BASE;

	fInput = (-1 * (SIGMOID_ERR) * (time - M)) / M;

	if (xSemaphore != NULL) {
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE) {

			p_euler->input = fInput;

			/* Wait for the euler block to become ready - this might hang!!!*/
					while (!p_euler->state){
						vTaskDelay(1);/*definitely 0, remove if redundant?*/
						//__asm("nop");
					}


			xSemaphoreGive( xSemaphore);
		} else {
			printf("Servo couldn't get EULERBLOCK semaphore! \n");
			// We could not obtain the semaphore and can therefore not access
			// the shared resource safely.
		}
	}

	*result = 1.0/(1+p_euler->output);

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
	unsigned int latency_ms = 0;
	float totaltime_ms;

	/* First work out how far we have to travel */
	pwm_get_pos(sData->iServoID, &initialposition);

	distance = (signed int) place - (signed int) initialposition;

	/* We've got no-where to go */
	if (abs(distance) < 10)
		return;


	/* This distance value will act as the scale factor for the Sigmoid function
	 * it's signed becuase we can of course be going in two directions */

	/* The speed for the transition is passed in so find the total time
	 * but it can't be negative. */
	totaltime = fabs((float) distance) / (float) speed;

	totaltime_ms = totaltime * 1000;

	/* M is the half-way point which is passed to the sigmoid function */
	m = totaltime_ms * 2;

	latency_ms = TICKS2MS(MOVE_SIGMOID_LATENCY);


	printf("Servo[%d] To[%d] Time[%d] Distance[%d] Init[%d] LatencyMS[%d]\n",
			sData->iServoID,
			place,
			(unsigned int)totaltime_ms,
			distance,
			initialposition,
			latency_ms);


	/* So start main loop */
	for (;;) {

		/* Quick message check */
		if (msg_recv_noblock(sData->qServo, &msgMessage) != ECD_NOMSG) {
			if (msgMessage.messageID == M_MOVE_STOP) {
				printf(
						"STOP message received before SIGMOID move finshed. Stopping. \n");
				return;
			} else {
				printf("Servo MID-SIGMOID-MOVE and received non-STOP message! Ignoring.\n");

			}
		}

		/* So no STOP message received, move one step */

		/* Get normalized value */
		sigmoid(5000, (float)latency_ms, &res);

		/* Now scale it */
		res *= distance;

		/* And add the initial offset */
		res += initialposition;

		/* Are we there yet? */
		if (latency_ms < (unsigned int)totaltime_ms) {

			/* Now move there */
			pwm_set_pos(sData->iServoID, (unsigned int) res);

			//n++;
			latency_ms += latency_ms;
		}
		else {

			printf("Servo [%d] Returns from sigmoid.\n",sData->iServoID);

			return;
		}

		vTaskDelay(MOVE_SIGMOID_LATENCY);

	} /* End of for loop */

} /* End of move sigmoid function */

