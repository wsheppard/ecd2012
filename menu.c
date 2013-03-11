#include "FreeRTOS.h"
#include "messaging.h"
#include "menu.h"
#include "ik.h"
#include "pwm.h"
#include "replay.h"

static void man_store_key_change(int key,int shifted, int replay_array_slot,int *M_MANMODE,
						int state, portTickType *xLastStateChange,portTickType *xNewStateChange);
static void man_enter_stopped_mode();

static int key_mappings[] = {14,9,10,11,5,6,7,1,2,3};
static xQueueHandle qMOVE;
static xQueueHandle qREPLAY;
static xQueueHandle qMENU;

int menu_init(xQueueHandle qMoveHandle,xQueueHandle qRHandle,xQueueHandle qMenuHandle){
	
	qMOVE=qMoveHandle;
	qREPLAY=qRHandle;
	qMENU=qMenuHandle;
	return ECD_OK;
}


/*This function implements the menu and returns 1 if the key pressed should be passed on to 
the movement functions*/
int man_check_menu(unsigned state, int shifted){
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
	replay_storage_s stateChangeValue;
	static int slot_key_binary=0;
	int x,key;
	ik_cart_pos_s centerIK;


	centerIK.x_pos = 29.16;
	centerIK.y_pos = 0;
	centerIK.z_pos = 0.3308;

	/*Convert to a binary representation of key pressed as 
	defined in messages.h*/
	key = 1<<shifted;
	if menu
	/**********************************************************************/
	/*******************on detecting a key pressed*************************/
	/**********************************************************************/
	/**********************************************************************/
	if(state & 1){
		if (M_MANMODE == M_MANMODE_STOPPED){
			if (mode_changed == 1){
				printf("Centering....\n");
				ik_calc_IK(qMOVE,centerIK);
				vTaskDelay(1000);
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
				man_enter_stopped_mode(&M_MANMODE,&mode_changed);
				return 0;
			}
			return 1;
		}
		/* Whilst in record mode fetch slot number to replay when mode first entered, otherwise log button
		presses in array and pass info onto movement modules */
		else if(M_MANMODE == M_MANMODE_RECORD){
			if (key==M_KP_KEY_C4){
				/*If mode was initilised and movements saved then save time recording ends */ 
				if (mode_changed != 1){
					replayMSG.messageID=REPLAY_STOP_RECORD;
					xNewStateChange=xTaskGetTickCount();
					stateChangeValue.delayTime=xNewStateChange-xLastStateChange;
					stateChangeValue.state=REPLAY_END;
					replayMSG.messageDATA=(unsigned int)&stateChangeValue;
					msg_send(qREPLAY,replayMSG);
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
						slot_key_binary=key_mappings[x];
					}
				}
			}
			/* If not stop button and slot has been selected log button presses */
			else{
				man_store_key_change(key,shifted,replay_array_slot,&M_MANMODE,REPLAY_BUTTON_DOWN,&xLastStateChange,&xNewStateChange);
				return 1;
			}

			return 0;
		}
		/*Whilst in replay mode always return 0 as replay task is controlling servos. Get slot from 
		user if mode has just been started. When stop button pressed send stop to replay task queue*/
		else if(M_MANMODE==M_MANMODE_REPLAY){
			if (key==M_KP_KEY_C4){
				replayMSG.messageID=REPLAY_STOP_PLAY;
				msg_send(qREPLAY,replayMSG);
				man_enter_stopped_mode(&M_MANMODE,&mode_changed);
				return 0;
			}
			/*Get Replay slot from user*/
			else if (mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (key==key_mappings[x]){
						mode_changed=0;
						ignore_slot_key_release=1;
						slot_key_binary=key_mappings[x];				
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
			/*Start up Record mode after slot choice button released*/
			if ((key == slot_key_binary) && (ignore_slot_key_release==1)){
				xNewStateChange = xTaskGetTickCount();
				printf("Recording Slot %d\n",replay_array_slot);
				replayMSG.messageID=REPLAY_START_RECORD;
				replayMSG.messageDATA=replay_array_slot;
				msg_send(qREPLAY,replayMSG);
				ignore_slot_key_release=0;
				slot_key_binary=0;
			}
			/* Log button releases in array along with time since last release */
			else if (mode_changed != 1){
				man_store_key_change(key,shifted,replay_array_slot,
					&M_MANMODE,REPLAY_BUTTON_UP,&xLastStateChange,&xNewStateChange);
				return 1;
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
		else if (M_MANMODE==M_MANMODE_REPLAY){
			if ((key == slot_key_binary) && (ignore_slot_key_release==1)){
			replayMSG.messageID=REPLAY_START_PLAY;
			replayMSG.messageDATA=x;
			msg_send(qREPLAY,replayMSG);
			printf("Replaying Slot %d\n",x);
			ignore_slot_key_release=0;
			slot_key_binary=0;
			}
		}
		return 0;
	}
	return 0;
}

void man_enter_stopped_mode(int *M_MANMODE,int *mode_changed){
	printf("Stopped\n");
	vTaskDelay(100);
	printf("Hit any key to\ncentre arm.\n");
	*mode_changed=1;
	*M_MANMODE=M_MANMODE_STOPPED;
	return;
}

void man_store_key_change(int key,int shifted, int replay_array_slot,int *M_MANMODE,
						int state, portTickType *xLastStateChange,portTickType *xNewStateChange){

	replay_storage_s stateChangeValue;
	msg_message_s replayMSG;
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
		replayMSG.messageID=REPLAY_RECORD;
		replayMSG.messageDATA=(unsigned int)&stateChangeValue;
		msg_send(qREPLAY,replayMSG);
	}
}
