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
#include "manager.h"
#include "pwm.h"
#include "display.h"
#include "ik.h"


/* Private Functions */
static void man_main(void*params);
static void man_key_down(int key);
static void man_key_up(int key);
static int check_menu(unsigned state, int key);
static void man_replay(void*params);

/* Queues */
static xQueueHandle qKP;
static xQueueHandle qMOVE;
static xQueueHandle qREPLAY;

/* Replay Storage Array */
static replay_storage_s replay_storage_array[NUM_REPLAY_SLOTS][NUM_REPLAY_STEPS];

int man_start(void){

	/* Create queue for keypad */
	msg_newQueue(&qKP);

	/* Start keypad task and give it the queue handle */
	kp_startTask(qKP);

	/* Create queue for movement module */
	msg_newQueue(&qMOVE);

	/* Start movement task and give it the queue handle */
	move_Start(qMOVE);

	/* Create queue for replay task */
	msg_newQueue(&qREPLAY);

	/* Start replay task */
	xTaskCreate( man_replay, "ReplayTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

	pwm_init();
	display_init();

	/* Now start the manager task */
	xTaskCreate( man_main, (fStr)"ManagerTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

	return ECD_OK;
}

/* Replay task that block waits on a queue till told to start. It then begins replaying and performing non blocking queries on the queue monitoring for
stop messages intermittently whilst performing the replay operations*/
static void man_replay(void*params){
	int replay_array_position;
	msg_message_s msgREPLAY;

	for(;;){
		msg_recv_block(qREPLAY,&msgREPLAY);
		if(msgREPLAY.messageID==REPLAY_START){
			//initiliase to start of replay array when starting a replay action
			replay_array_position=0;
			//Keep sending commands whilst there are still replay steps left, not end of array and have not recieved stop message
			while((replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state !=REPLAY_END) && (replay_array_position != (NUM_REPLAY_STEPS+1)) && (msgREPLAY.messageID!=REPLAY_STOP)){
				//Non blocking wait on Stop messages on replay queue
				msg_recv_noblock(qREPLAY,&msgREPLAY);
				//Delay task until next event due 
				vTaskDelay(replay_storage_array[msgREPLAY.messageDATA][replay_array_position].delayTime);
				//TODO:This method could result in a larg(ish) period of time between pressing stop and actually stopping with a maximum of the time taken to 
				//fully move a pwm from a max to min position to overcome this we could split the delay into suitable sized chunks and check intermittently
				//for stop messages. Either that or we could implement some other way of directly stopping the pwms.
				if (replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state == REPLAY_BUTTON_UP){
					man_key_up(replay_storage_array[msgREPLAY.messageDATA][replay_array_position].keyPressed);
				}
				else if (replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state == REPLAY_BUTTON_DOWN){
					man_key_down(replay_storage_array[msgREPLAY.messageDATA][replay_array_position].keyPressed);
				}
				replay_array_position++;
			}
			msgREPLAY.messageID=REPLAY_STOP;
		}
	}
}

//This function implements the menu and returns 1 if the key pressed should be passed on to the movement modules
static int check_menu(unsigned state, int shifted){
	//stop message for use when entering stopped mode
	static msg_message_s msgSTOP = {M_MOVE_STOP,0};
	static msg_message_s replayMSG;
	//Flag signifying emu mode has recently changed
	static int mode_changed=0;
	static int ignore_release=0;
	//initiallising emu mode to stopped to start
	static int M_MANMODE = M_MANMODE_STOPPED;
	replay_storage_s stateChangeValue;
	static int replay_array_slot,replay_array_position;
	static int replay_start=0;
	static portTickType xLastStateChange,xNewStateChange;
	int x,key;


	key = 1<<shifted;
	//Perform action based on keypad input pressed and current mode of operation
	//perform this set of actions on detecting a button pressed
	if(state & 1){
		if (M_MANMODE == M_MANMODE_STOPPED){
			if (mode_changed==1){
				//Send stop to all servo's when entering stopped mode mode.
				for (x=0;x<PWM_COUNT;x++){
					msgSTOP.messageDATA = x;
					msg_send(qMOVE,msgSTOP);
				}
				mode_changed=0;
			}
			if (key == M_KP_KEY_C1){
				printf("Manual Control\n");
				mode_changed = 1;
				M_MANMODE=M_MANMODE_CONTROL;
			}
			else if (key == M_KP_KEY_C2){
				printf("Record - Enter a\nslot from 0-%d",NUM_REPLAY_SLOTS);
				ignore_release=1;
				mode_changed=1;
				M_MANMODE=M_MANMODE_RECORD;
			}
			else if (key == M_KP_KEY_C3){
				printf("Replay - Enter a\nreplay slot from 0-%d",NUM_REPLAY_SLOTS);
				mode_changed=1;
				M_MANMODE=M_MANMODE_REPLAY;
			}
			else if (key == M_KP_KEY_D1){
				printf("Centering Servo's");
				//TODO: In stopped mode this key will center emu. SHould think about emergency stop whilst centering!?
			}
			return 0;
		}
		//Whilst in manual control return 1 to tell calling function to pass key input onto movement module
		//unless the stop button is pressed
		else if( M_MANMODE == M_MANMODE_CONTROL){
			if (key==M_KP_KEY_C4){
				mode_changed=1;
				M_MANMODE=M_MANMODE_STOPPED;
				return 0;
			}
			return 1;
		}
		else if(M_MANMODE == M_MANMODE_RECORD){
			if (key==M_KP_KEY_C4){
				printf("Stopped");
				M_MANMODE=M_MANMODE_STOPPED;
				mode_changed=1;
				return 0;
			}
			//If just started replay mode get slot to use from user
			else if(mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (key==x){
						replay_array_slot=x-1;
						ignore_release=0;
						//TODO:Get and store current position of PWM's as first value in array if not centered.
						replay_array_position=0;
						xNewStateChange = xTaskGetTickCount();
						printf("Recording Slot %d",replay_array_slot);
						return 0;
					}
				}
			}
			//Otherwise log button presses.
			else{
				//TODO: Check overflow time limits before 32bit value rolls over and either make an ERROR if they can be 
				//reasonably reached or support a rollover and rollover counter.
				xLastStateChange=xNewStateChange;
				xNewStateChange=xTaskGetTickCount();
				stateChangeValue.keyPressed=shifted;
				stateChangeValue.delayTime=xNewStateChange-xLastStateChange;
				stateChangeValue.state=REPLAY_BUTTON_DOWN;
				replay_storage_array[replay_array_slot][replay_array_position]=stateChangeValue;
				replay_array_position++;
				//Detect if replay is too large for storage
				if (replay_array_position == NUM_REPLAY_STEPS){
					printf("Error:Slot mem\nlimit Reached\n");
					M_MANMODE=M_MANMODE_STOPPED;
					vTaskDelay(1000);
					printf("Stopped");
					return 0;
				}
				return 1;
			}
		}
		else if(M_MANMODE==M_MANMODE_REPLAY){
			if (key==M_KP_KEY_C4){
				replayMSG.messageID=REPLAY_STOP;
				msg_send(qREPLAY,replayMSG);
				mode_changed=1;
				printf("Stopped");
				M_MANMODE=M_MANMODE_STOPPED;
				return 0;
			}
			//Get Replay slot from user
			if (mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (key==x){
						mode_changed=0;
						replayMSG.messageID=REPLAY_START;
						replayMSG.messageDATA=x-1;
						msg_send(qREPLAY,replayMSG);
						printf("Replaying Slot %d",x);
						return 0;						
					}
				}
			}
		}
	}
	//perform these actions on detecting a key released
	else{
		if (M_MANMODE==M_MANMODE_RECORD){
			if (mode_changed != 1){
				xLastStateChange=xNewStateChange;
				xNewStateChange=xTaskGetTickCount();
				stateChangeValue.keyPressed=shifted;
				stateChangeValue.delayTime=xNewStateChange-xLastStateChange;
				stateChangeValue.state=REPLAY_BUTTON_UP;
				replay_storage_array[replay_array_slot][replay_array_position]=stateChangeValue;
				replay_array_position++;
				if (replay_array_position == NUM_REPLAY_STEPS){//need to test this
					printf("Error:Slot mem\nlimit Reached\n");
					M_MANMODE=M_MANMODE_STOPPED;
					vTaskDelay(1000);
					return 0;
				}
				return 1;
			}
			//Ignore the release of the slot selection and mode selection keys immediately after user choices are made
			else if((key == replay_array_slot+1) && (ignore_release!=1)){
				mode_changed=0;
				return 0;
			}
		}
		else if (M_MANMODE==M_MANMODE_CONTROL){
			if(mode_changed == 1){
				mode_changed=0;
				return 0;
			}
			return 1;
		}
		return 0;
	}
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

	startIK.x_pos = 10;
	startIK.y_pos = 10;
	startIK.z_pos = 0;

	stopIK.x_pos = 12;
	stopIK.y_pos = 12;
	stopIK.z_pos = 0;

	for (;;) {
	
		/* Block wait on KEYPAD input - maybe this should be on timeout for debug 
			purposes? */
		msg_recv_block(qKP,&msgKP);

		printf("Received a message... ID: %d, DATA: 0x%X.\n", msgKP.messageID, msgKP.messageDATA);
	
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
						if (check_menu(state,shifted) == 1){
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

				}
				else{
						printf("Key at pos %d released.\n", shifted);
						if (check_menu(state,shifted) == 1){
						man_key_up(shifted);
						}

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
	}
}
