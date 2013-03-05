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
static int man_check_menu(unsigned state, int key);
static int man_store_key_change(int key,int shifted, int replay_array_slot, int *replay_array_position,int *M_MANMODE,
						int *mode_changed,int state, portTickType *xLastStateChange,portTickType *xNewStateChange);
static void man_stop_all_pwm();
static void man_enter_stopped_mode();
static void man_replay(void*params);

/* Queues */
static xQueueHandle qKP;
static xQueueHandle qMOVE;
static xQueueHandle qREPLAY;

/* Replay Storage Array */
static replay_storage_s replay_storage_array[NUM_REPLAY_SLOTS][NUM_REPLAY_STEPS];
static int key_mappings[] = {14,9,10,11,5,6,7,1,2,3};

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
	xTaskCreate( man_replay, "ReplayTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	pwm_init();
	display_init();

	/* Now start the manager task */
	xTaskCreate( man_main, (fStr)"ManagerTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

	return ECD_OK;
}

/* Replay task that block waits on a queue till told to start. It then begins replaying and performing non blocking queries on the queue monitoring for
stop messages intermittently whilst performing the replay operations*/
static void man_replay(void*params){
	int replay_array_position,num_delays,leftover_time;
	msg_message_s msgREPLAY;

	for(;;){
		msg_recv_block(qREPLAY,&msgREPLAY);
		if(msgREPLAY.messageID==REPLAY_START){
			/*initiliase to start of replay array when starting a replay action*/
			replay_array_position=0;
			/*Keep sending commands whilst there are still replay steps left, not end of array and have not recieved stop message*/
			while((replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state !=REPLAY_END) && 
										(replay_array_position != (NUM_REPLAY_STEPS+1)) && (msgREPLAY.messageID!=REPLAY_STOP)){
				/*Non blocking wait on Stop messages on replay queue*/
				msg_recv_noblock(qREPLAY,&msgREPLAY);
				/* A single delay the full length of the period between keypad state changes could result in a larg(ish) period of time between pressing stop
				and the replay actually stopping. To overcome this I've split the delay into suitable sized chunks and check intermittently for stop messages. 
				It's either this or implementing some other way of directly stopping the pwms. E.G an interrupt */
				num_delays=replay_storage_array[msgREPLAY.messageDATA][replay_array_position].delayTime / STOP_POLL_DELAY;
				leftover_time=replay_storage_array[msgREPLAY.messageDATA][replay_array_position].delayTime % STOP_POLL_DELAY;
				while(num_delays){
					vTaskDelay(STOP_POLL_DELAY);
					msg_recv_noblock(qREPLAY,&msgREPLAY);
					if (msgREPLAY.messageID==REPLAY_STOP)
						break;
					num_delays--;
				}
				if (msgREPLAY.messageID==REPLAY_STOP)
					break;
				vTaskDelay(leftover_time);
				if (replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state == REPLAY_BUTTON_UP){
					man_key_up(replay_storage_array[msgREPLAY.messageDATA][replay_array_position].keyPressed);
				}
				else if (replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state == REPLAY_BUTTON_DOWN){
					man_key_down(replay_storage_array[msgREPLAY.messageDATA][replay_array_position].keyPressed);
				}
				replay_array_position++;
			}
			if (replay_storage_array[msgREPLAY.messageDATA][replay_array_position].state ==REPLAY_END){
				printf("Replay finished\nSucessfully\n");
			}
			else if (msgREPLAY.messageID==REPLAY_STOP){
				printf("Replay was\nstopped\n");
			}
			/* When end of saved replay is reached set msgReplay to stop so that it does not re-enter play loop*/
			msgREPLAY.messageID=REPLAY_STOP;
			/* Send stop message to all pwms in case of forced stop message */
			man_stop_all_pwm();
		}
	}
}

static void man_main(void*params){

	msg_message_s msgKP;
	ik_cart_pos_s startIK; //temporary until we have a module feeding the IK with position data
	ik_cart_pos_s stopIK;
	ik_cart_pos_s current_pos;


	unsigned changed;
	unsigned state;
	int shifted;
	int do_ik_once = 1;

	printf("Starting manager...\n");

	startIK.x_pos = 15.13;
	startIK.y_pos = 15.13;
	startIK.z_pos = 16.99;

	stopIK.x_pos = 19.53;
	stopIK.y_pos = -19.53;
	stopIK.z_pos = -9.08;

	ik_cart_pos_s centerIK;


	centerIK.x_pos = 29.16;
	centerIK.y_pos = 0;
	centerIK.z_pos = 0.3308;

	ik_calc_IK(qMOVE,centerIK);


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
						if (man_check_menu(state,shifted) == 1){
						man_key_down(shifted);
						if(shifted == 8){
							if(do_ik_once){
								ik_calc_FK(&current_pos);
								printf("current_pos: x = %f, y = %f, z = %f\n",current_pos.x_pos,current_pos.y_pos,current_pos.z_pos);

								printf("Calculate inverse kinematics for the start position: x = %f, y = %f, z = %f\n",startIK.x_pos,startIK.y_pos,startIK.z_pos);
						    	ik_calc_IK(qMOVE,startIK);
								do_ik_once = 0;
							}
						}
						if(shifted == 12){
								if(do_ik_once == 0){
									ik_calc_FK(&current_pos);
									printf("current_pos: x = %f, y = %f, z = %f\n",current_pos.x_pos,current_pos.y_pos,current_pos.z_pos);

									printf("Calculate inverse kinematics for the stop position: x = %f, y = %f, z = %f\n",stopIK.x_pos,stopIK.y_pos,stopIK.z_pos);
							    	ik_calc_IK(qMOVE,stopIK);
									do_ik_once = 1;
								}
							}
				}

				}
				else{
						printf("Key at pos %d released.\n", shifted);
						if (man_check_menu(state,shifted) == 1){
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

/*This function implements the menu and returns 1 if the key pressed should be passed on to 
the movement functions*/
static int man_check_menu(unsigned state, int shifted){
	/*stop message for use when entering stopped mode*/
	static msg_message_s replayMSG;
	/*Flag signifying emu mode has recently changed*/
	static int mode_changed=0;
	static int ignore_slot_key_release=0;
	/*initiallising emu mode to stopped on first call*/
	static int M_MANMODE = M_MANMODE_STOPPED;
	static int replay_array_slot,replay_array_position;
	static int replay_start=0;
	static portTickType xLastStateChange, xNewStateChange;
	static int slot_key_binary=0;
	int x,key;
	ik_cart_pos_s centerIK;


	centerIK.x_pos = 29.16;
	centerIK.y_pos = 0;
	centerIK.z_pos = 0.3308;

	/*Convert to a binary representation of key pressed as 
	defined in messages.h*/
	key = 1<<shifted;
	/**********************************************************************/
	/*******************on detecting a key pressed*************************/
	/**********************************************************************/
	/**********************************************************************/
	if(state & 1){
		if (M_MANMODE == M_MANMODE_STOPPED){
			if (mode_changed == 1){
				printf("Centering....\n");
				/* 
				put specific 
				move to centre code here 
				*/
				ik_calc_IK(qMOVE,centerIK);

				printf("Stopped\n");
				mode_changed=0;
			}
			else if (key == M_KP_KEY_C1){
				printf("Manual Control\n");
				mode_changed = 1;
				M_MANMODE=M_MANMODE_CONTROL;
			}
			else if (key == M_KP_KEY_C2){
				printf("Record - Enter a\nslot from 0-%d\n",NUM_REPLAY_SLOTS);
				mode_changed=1;
				M_MANMODE=M_MANMODE_RECORD;
			}
			else if (key == M_KP_KEY_C3){
				printf("Replay - Enter a\nreplay slot from 0-%d\n",NUM_REPLAY_SLOTS);
				mode_changed=1;
				M_MANMODE=M_MANMODE_REPLAY;
			}
			return 0;
		}
		/*Whilst in manual control always return 1 to tell calling function to pass key input onto movement 
		functions unless the stop button is pressed*/
		else if( M_MANMODE == M_MANMODE_CONTROL){
			if (key==M_KP_KEY_C4){
				man_stop_all_pwm();
				man_enter_stopped_mode(&M_MANMODE,&mode_changed);
				return 0;
			}
			return 1;
		}
		/* Whilst in record mode fetch slot number to replay when mode first entered, otherwise log button
		presses in array and pass info onto movement modules */
		else if(M_MANMODE == M_MANMODE_RECORD){
			if (key==M_KP_KEY_C4){
				man_stop_all_pwm();
				/*If mode was initilised and movements saved then save time recording end */ 
				if (mode_changed != 0){
					/* The key press used below does not matter. The binary shifted value given to the movement functions
					is not in their range and will just be ignored. It is only submitted to ensure that replay continues
					until the final stored time*/
					man_store_key_change(M_KP_KEY_A1,0,replay_array_slot,&replay_array_position,&M_MANMODE,
						&mode_changed,REPLAY_BUTTON_DOWN,&xLastStateChange,&xNewStateChange);
				}
				man_enter_stopped_mode(&M_MANMODE,&mode_changed);
				return 0;
			}
			/*If just started replay mode get slot from user*/
			else if(mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (key==key_mappings[x]){
						/* set mode changed to 0 so key state changes can start being logged*/
						mode_changed=0;
						replay_array_slot=x;
						ignore_slot_key_release=1;
						/* Possible TODO:get and store current position of PWM's as first value in array if not centered.
						and use specific move to that position at start of replay */
						replay_array_position=0;
						xNewStateChange = xTaskGetTickCount();
						printf("Recording Slot %d\n",replay_array_slot);
						slot_key_binary=key_mappings[x];
					}
				}
			}
			/* If not stop button and slot has been selected log button presses */
			else{
			return man_store_key_change(key,shifted,replay_array_slot,&replay_array_position,&M_MANMODE,&mode_changed,REPLAY_BUTTON_DOWN,&xLastStateChange,&xNewStateChange);
			}

			return 0;
		}
		/*Whilst in replay mode always return 0 as replay task is controlling servos. Get slot from 
		user if mode has just been started. When stop button pressed send stop to replay task queue*/
		else if(M_MANMODE==M_MANMODE_REPLAY){
			if (key==M_KP_KEY_C4){
				replayMSG.messageID=REPLAY_STOP;
				msg_send(qREPLAY,replayMSG);
				man_enter_stopped_mode(&M_MANMODE,&mode_changed);
				return 0;
			}
			/*Get Replay slot from user*/
			else if (mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (key==key_mappings[x]){
						mode_changed=0;
						replayMSG.messageID=REPLAY_START;
						replayMSG.messageDATA=x;
						msg_send(qREPLAY,replayMSG);
						printf("Replaying Slot %d\n",x);					
					}
				}
			}
			return 0;	
		}
	}
	/**********************************************************************/
	/*******************on detecting a key released************************/
	/**********************************************************************/
	/**********************************************************************/
	else{
		if (M_MANMODE==M_MANMODE_RECORD){
			/*Ignore the release of the slot selection immediately after user choices are made*/
			if ((key == slot_key_binary) && (ignore_slot_key_release==1)){
				ignore_slot_key_release=0;
				slot_key_binary=0;
			}
			/* Log button releases in array along with time since last release */
			else if (mode_changed != 1){
				return man_store_key_change(key,shifted,replay_array_slot,&replay_array_position,
					&M_MANMODE,&mode_changed,REPLAY_BUTTON_UP,&xLastStateChange,&xNewStateChange);
			}
		}
		else if (M_MANMODE==M_MANMODE_CONTROL){
			/* wait till user lets go of manual control button before starting movement */
			if(mode_changed == 1 && (key ==M_KP_KEY_C1)){
				mode_changed=0;
				return 0;
			}
			return 1;
		}
		return 0;
	}
	return 0;
}

void man_stop_all_pwm(){
	int x;
	msg_message_s msgSTOP = {M_MOVE_STOP,0};

	for (x=0;x<PWM_COUNT;x++){
		msgSTOP.messageDATA = x;
		msg_send(qMOVE,msgSTOP);
	}
	return;
}

void man_enter_stopped_mode(int *M_MANMODE,int *mode_changed){
	printf("Stopped\n");
	vTaskDelay(100);
	printf("Hit any key to\ncentre arm.\n");
	*mode_changed=1;
	*M_MANMODE=M_MANMODE_STOPPED;
	return;
}

int man_store_key_change(int key,int shifted, int replay_array_slot, int *replay_array_position,int *M_MANMODE,
						int *mode_changed,int state, portTickType *xLastStateChange,portTickType *xNewStateChange){
	replay_storage_s stateChangeValue;
	/*TODO: Check overflow time limits before 32bit value rolls over and either make an ERROR if they can be 
	reasonably reached or support a rollover and rollover counter.*/
	switch (key){
	case M_KP_KEY_A1:
	case M_KP_KEY_A2:
	case M_KP_KEY_A3:
	case M_KP_KEY_A4:
	case M_KP_KEY_B1:
	case M_KP_KEY_B2:
	case M_KP_KEY_B3:
	case M_KP_KEY_B4:
		*xLastStateChange=*xNewStateChange;
		*xNewStateChange=xTaskGetTickCount();
		stateChangeValue.keyPressed=shifted;
		stateChangeValue.delayTime=*xNewStateChange-*xLastStateChange;
		stateChangeValue.state=state;
		replay_storage_array[replay_array_slot][*replay_array_position]=stateChangeValue;
		(*replay_array_position)+=1;
		if (*replay_array_position == NUM_REPLAY_STEPS){
			man_stop_all_pwm();
			printf("Error:Slot mem\nlimit Reached\n");
			vTaskDelay(100);
			man_enter_stopped_mode(M_MANMODE,mode_changed);
			return 0;
		}
		return 1;
	default:
		return 0;
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
