/* Manager thread. Looking after the basic control of the whole system. Marshalling of inputs
	to the correct output context. */


/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Local includes */
#include "messaging.h"
#include "keypad.h"
#include "movement.h"
#include "pwm.h"
#include "display.h"
#include "ik.h"


/* Private Functions */
static void man_main(void*params);
static void man_key_down(int key);
static void man_key_up(int key);

/* Queues */
static xQueueHandle qKP;
static xQueueHandle qMOVE;

int man_start(void){

	/* Create queue for keypad */
	msg_newQueue(&qKP);

	/* Start keypad task and give it the queue handle */
	kp_startTask(qKP);

	/* Create queue for movement module */
	msg_newQueue(&qMOVE);

	/* Start movement task and give it the queue handle */
	move_Start(qMOVE);

	pwm_init();
	display_init();

	/* Now start the manager task */
	xTaskCreate( man_main, (fStr)"ManagerTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

	return ECD_OK;
}

static void man_main(void*params){

	msg_message_s msgKP;
	ik_cart_pos_s startIK; //temporary until we have a module feeding the IK with position data
	ik_cart_pos_s stopIK;
	unsigned changed;
	unsigned state;
	int shifted;
	int do_ik_once = 1;

	printf("Starting manager...\n");

	startIK.x_pos = 20.62;
	startIK.y_pos = 20.62;
	startIK.z_pos = -4.846;

	stopIK.x_pos = 20.62;
	stopIK.y_pos = -20.62;
	stopIK.z_pos = -4.846;

	for (;;) {
	
		/* Block wait on KEYPAD input - maybe this should be on timeout for debug 
			purposes? */
		msg_recv_block(qKP,&msgKP);

		//printf("Received a message... ID: %d, DATA: 0x%X.\n", msgKP.messageID, msgKP.messageDATA);
	
		changed = 65535U & msgKP.messageDATA;
		state = msgKP.messageDATA >> 16;
		shifted = 0;

		//printf("Changed:[0x%X], State[0x%X].\n", changed, state);

		/* While there are still bits to deal with */
		while(changed){
		
			/* The key at this postion is changed */
			if (changed & 1){
			
				if(state & 1){

						printf("Key at pos %d pressed.\n", shifted);			
						man_key_down(shifted);
						if(shifted == 8){
							if(do_ik_once){
						    	printf("Calculate inverse kinematics for the start position.\n");

								ik_calc_IK(qMOVE,startIK);
								do_ik_once = 0;
							}
						}
						if(shifted == 12){
								if(do_ik_once == 0){
							    	printf("Calculate inverse kinematics for the stop position.\n");

									ik_calc_IK(qMOVE,stopIK);
									do_ik_once = 1;
								}
							}

				}
				else{
						printf("Key at pos %d released.\n", shifted);
						man_key_up(shifted);

				}
			
			}
			
			changed >>= 1;
			state >>= 1;
			shifted++;
			
		}

	
	}
	
}

/* This just sends a message, it shouldn't hang around */
void man_key_down(int key){
	
	msg_message_s msgMessage;

	/* Put it back into binary format */
	key = 1<<key;

	switch (key){
	case M_KP_KEY_A1:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO1 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_A2:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO2 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_A3:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO3 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_A4:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO4 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		break;
	
	case M_KP_KEY_B1:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO1;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_B2:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO2;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_B3:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO3;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_B4:
		msgMessage.messageID = M_MOVE_CONT;
		msgMessage.messageDATA = M_MOVE_SERVO4;
		msg_send(qMOVE,msgMessage);
		break;
	
	/* Save / replay functions */
		/* NOTE: These don't need up and down - so only really UP is used */
	case M_KP_KEY_C1:
		/* Save current position */	
		break;
	
	}

}


void man_key_up(int key){


	msg_message_s msgMessage;

	/* Put it back into binary format */
	key = 1<<key;

	switch (key){
	case M_KP_KEY_A1:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO1;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_A2:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO2;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_A3:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO3;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_A4:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO4;
		msg_send(qMOVE,msgMessage);
		break;


	case M_KP_KEY_B1:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO1;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_B2:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO2;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_B3:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO3;
		msg_send(qMOVE,msgMessage);
		break;
	case M_KP_KEY_B4:
		msgMessage.messageID = M_MOVE_STOP;
		msgMessage.messageDATA = M_MOVE_SERVO4;
		msg_send(qMOVE,msgMessage);
		break;


	/* Save position */
#if 0
	case M_KP_KEY_c1:
		msgMessage.messageID = 
#endif

	}
}
