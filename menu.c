/******************************************************************************
*
*       File: menu.c
*       Language: c
*	AUTHOR: Joshua Baxter
*       E-Mail: joshuagbaxter<at>gmail.com
*       git:https://github.com/wsheppard/ecd2012/
*       
*
* Description:  Menu module with functions for taking user input and performing
* 		the appropriate actions.
*******************************************************************************/
#include "FreeRTOS.h"
#include "messaging.h"
#include "menu.h"
#include "ik.h"
#include "pwm.h"
#include "replay.h"

static void men_store_key_change(int key,int shifted, int state, portTickType *xLastStateChange,
	portTickType *xNewStateChange);
static void men_enter_stopped_mode();
static int men_ik_control(int key);

static int key_mappings[] = {11,12,8,4,13,9,5,14,10,6};
//static int key_mappings[] = {14,9,10,11,5,6,7,1,2,3};
static xQueueHandle qMOVE;
static xQueueHandle qREPLAY;
static xQueueHandle qMENU;

int menu_init(xQueueHandle qMoveHandle,xQueueHandle qRHandle,xQueueHandle qMenuHandle){
	
	qMOVE=qMoveHandle;
	qREPLAY=qRHandle;
	qMENU=qMenuHandle;
	printf( M_POS1_1 M_CLEAR_LINE "Menu: 1:WP 2:MAN");
	printf( M_POS2_1 M_CLEAR_LINE "4:IK 5:RT 8:RP");
	return ECD_OK;

}


/*This function implements the menu and returns 1 if the key pressed should be passed on to 
the movement functions*/
int men_check_menu(unsigned state, int shifted){
	/*stop message for use when entering stopped mode*/
	static msg_message_s replayMSG;
	/*Flag signifying emu mode has recently changed*/
	static int mode_changed=0;
	static int ignore_slot_key_release=0;
	/*initiallising emu mode to stopped on first call*/
	static int M_MENMODE = M_MENMODE_STOPPED;
	static int replay_array_slot;
	static int replay_start=0;
	static portTickType xLastStateChange, xNewStateChange;
	replay_storage_s stateChangeValue;
	static int waypoint_pos=0;
	static int slot_key_binary=0;
	int x,key;
	static ik_cart_pos_s centerIK,current_pos_ik;


	centerIK.x_pos = 29.16;
	centerIK.y_pos = 0;
	centerIK.z_pos = 0.3308;

	/*Convert to a binary representation of key pressed as 
	defined in messages.h*/
	key = 1<<shifted;
	//if menu error in menu queue.
	/**********************************************************************/
	/*******************on detecting a key pressed*************************/
	/**********************************************************************/
	/**********************************************************************/

	if(state & 1){

		if (M_MENMODE == M_MENMODE_STOPPED){
			if (mode_changed == 1){
				printf(M_CLEAR_SCREEN "Centering....\n\n");
				vTaskDelay(MS2TICKS(ik_move_goal(centerIK)));
				pwm_set_pos(0, 75000);
				printf("Stopped\n\n");
				printf( M_POS1_1 M_CLEAR_LINE "Menu: 1:WP 2:MAN");
				printf( M_POS2_1 M_CLEAR_LINE "4:IK 5:RT 8:RP");
				mode_changed=0;
			}
			else if (key == M_KP_KEY_C1){
				printf(M_POS1_1 M_CLEAR_SCREEN "Manual Control");
				mode_changed = 1;
				M_MENMODE=M_MENMODE_CONTROL;
			}
			else if (key == M_KP_KEY_D2){
				printf(M_POS1_1 M_CLEAR_SCREEN "IK Control");
				mode_changed = 1;
				M_MENMODE=M_MENMODE_CONTROL_IK;
			}
			else if (key == M_KP_KEY_C2){
				printf(M_POS1_1 M_CLEAR_SCREEN "RTRecord - Enter\na slot from 1-9");
				mode_changed=1;
				M_MENMODE=M_MENMODE_RECORD_RT;
			}
			else if (key == M_KP_KEY_D1){
				printf( M_POS1_1 M_CLEAR_SCREEN "WPRecord - Enter\na slot from 1-9");
				mode_changed=1;
				M_MENMODE=M_MENMODE_RECORD_WP;
			}
			else if (key == M_KP_KEY_C3){
				printf(M_POS1_1 M_CLEAR_SCREEN "Replay - Enter a\na slot from 1-9");
				mode_changed=1;
				M_MENMODE=M_MENMODE_REPLAY;
			}
			return 0;
		}
		/*Whilst in manual control always return 1 to tell calling function to pass key input onto movement 
		functions unless the stop button is pressed*/
		else if( M_MENMODE == M_MENMODE_CONTROL){
			if (key==M_KP_KEY_C4){
				men_enter_stopped_mode(&M_MENMODE,&mode_changed);
				return 0;
			}
			return 1;
		}
		else if( M_MENMODE == M_MENMODE_CONTROL_IK){
			if (key==M_KP_KEY_C4){
				men_enter_stopped_mode(&M_MENMODE,&mode_changed);
				return 0;
			}
			return men_ik_control(key);
		}
		/* Whilst in record mode fetch slot number to replay when mode first entered, otherwise log button
		presses in array and pass info onto movement modules */
		else if(M_MENMODE == M_MENMODE_RECORD_RT){
			if (key==M_KP_KEY_C4){
				/*If mode was initilised and movements saved then save time recording ends */ 
				if (mode_changed != 1){
					replayMSG.messageID=REPLAY_STOP_RECORD;
					xNewStateChange=xTaskGetTickCount();
					stateChangeValue.delayTime=xNewStateChange-xLastStateChange;
					stateChangeValue.state=REPLAY_END;
					replayMSG.messageDATA=(unsigned int)&stateChangeValue;
					msg_send(qREPLAY,replayMSG);
					vTaskDelay((5/portTICK_RATE_MS));		
				}
				men_enter_stopped_mode(&M_MENMODE,&mode_changed);
				return 0;
			}
			/*If just started replay mode get slot from user*/
			else if(mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (shifted==key_mappings[x]){
						/* set mode changed to 0 so key state changes can start being logged*/
						mode_changed=0;
						replay_array_slot=x;
						ignore_slot_key_release=1;
						/* Possible TODO:get and store current position of PWM's as first value in array if not centered.
						and use specific move to that position at start of replay */
						slot_key_binary=key;
					}
				}
			}
			/* If not stop button and slot has been selected log button presses */
			else{
				men_store_key_change(key,shifted,REPLAY_BUTTON_DOWN,&xLastStateChange,&xNewStateChange);
				return 1;
			}

			return 0;
		}
		else if(M_MENMODE == M_MENMODE_RECORD_WP){
			if (key==M_KP_KEY_C4){
				/*If mode was initilised and movements saved then save time recording ends */ 
				if (mode_changed != 1){
					replayMSG.messageID=REPLAY_STOP_RECORD;
					stateChangeValue.state=REPLAY_END;
					replayMSG.messageDATA=(unsigned int)&stateChangeValue;
					msg_send(qREPLAY,replayMSG);
					vTaskDelay((5/portTICK_RATE_MS));		
				}
				men_enter_stopped_mode(&M_MENMODE,&mode_changed);
				return 0;
			}
			/*If just started replay mode get slot from user*/
			else if(mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (shifted==key_mappings[x]){
						/* set mode changed to 0 so key state changes can start being logged*/
						mode_changed=0;
						replay_array_slot=x;
						ignore_slot_key_release=1;
						slot_key_binary=key;
						waypoint_pos = 0;
					}
				}
			}
			/* If not stop button and slot has been selected log button presses */
			else if ( key == M_KP_KEY_D4){
				ik_calc_FK(&current_pos_ik);
				stateChangeValue.ik_position=current_pos_ik;
				pwm_get_pos(0,&stateChangeValue.rValue);
				if (stateChangeValue.rValue> 100000)
					stateChangeValue.rValue=100000;
				else if (stateChangeValue.rValue< 50000)
					stateChangeValue.rValue = 50000;
				stateChangeValue.state=REPLAY_WP;
				replayMSG.messageID=REPLAY_RECORD;
				replayMSG.messageDATA=(unsigned int)&stateChangeValue;
				msg_send(qREPLAY,replayMSG);
				waypoint_pos++;
				printf( M_POS1_1 M_CLEAR_SCREEN "Waypoint Record\nSlot %d, pos %d\n",replay_array_slot,waypoint_pos);
				vTaskDelay((5/portTICK_RATE_MS));				
			}
			else {
				return 1;
			}
			return 0;
		}
		/*Whilst in replay mode always return 0 as replay task is controlling servos. Get slot from 
		user if mode has just been started. When stop button pressed send stop to replay task queue*/
		else if(M_MENMODE==M_MENMODE_REPLAY){
			if (key==M_KP_KEY_C4){
				replayMSG.messageID=REPLAY_STOP_PLAY;
				msg_send(qREPLAY,replayMSG);
				vTaskDelay(1000);
				vTaskDelay((3/portTICK_RATE_MS));		
				men_enter_stopped_mode(&M_MENMODE,&mode_changed);
				return 0;
			}
			/*Get Replay slot from user*/
			else if (mode_changed==1){
				for (x=0;x<NUM_REPLAY_SLOTS;x++){
					if (shifted==key_mappings[x]){
						mode_changed=0;
						replay_array_slot=x;
						ignore_slot_key_release=1;
						slot_key_binary=key;
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
		if (M_MENMODE==M_MENMODE_RECORD_RT){
			/*Start up Record mode after slot choice button released*/
			if ((key == slot_key_binary) && (ignore_slot_key_release==1)){
				xNewStateChange = xTaskGetTickCount();
				printf( M_POS1_1 M_CLEAR_SCREEN "RealTime Record\nSlot %d\n",replay_array_slot);
				replayMSG.messageID=REPLAY_START_RECORD;
				replayMSG.messageDATA=replay_array_slot;
				msg_send(qREPLAY,replayMSG);
				vTaskDelay((3/portTICK_RATE_MS));
				ignore_slot_key_release=0;
				slot_key_binary=0;
			}
			/* Log button releases in array along with time since last release */
			else if (mode_changed != 1){
				men_store_key_change(key,shifted,
					REPLAY_BUTTON_UP,&xLastStateChange,&xNewStateChange);
				return 1;
			}
		}
		if (M_MENMODE==M_MENMODE_RECORD_WP){
			/*Start up Record mode after slot choice button released*/
			if ((key == slot_key_binary) && (ignore_slot_key_release==1)){
				printf( M_POS1_1 M_CLEAR_SCREEN "Waypoint Record\nSlot %d, pos 0\n",replay_array_slot);
				replayMSG.messageID=REPLAY_START_RECORD_WP;
				replayMSG.messageDATA=replay_array_slot;
				msg_send(qREPLAY,replayMSG);
				vTaskDelay((3/portTICK_RATE_MS));
				ignore_slot_key_release=0;
				slot_key_binary=0;
			}
			/* Log button releases in array along with time since last release */
			else if (mode_changed != 1){
				return 1;
			}
		}
		else if (M_MENMODE==M_MENMODE_CONTROL){
			/* wait till user lets go of manual control button before starting movement */
			if(mode_changed == 1 && (key ==M_KP_KEY_C1)){
				mode_changed=0;
				return 0;
			}
			return 1;
		}
		else if (M_MENMODE==M_MENMODE_CONTROL_IK){
			/* wait till user lets go of manual control button before starting movement */
			if(mode_changed == 1 && (key ==M_KP_KEY_C1)){
				mode_changed=0;
				return 0;
			}
			if (( key == M_KP_KEY_A1) || (key == M_KP_KEY_B1)){
				return 1;
			}
			return 0;
		}
		else if (M_MENMODE==M_MENMODE_REPLAY){
			if ((key == slot_key_binary) && (ignore_slot_key_release==1)){
			replayMSG.messageID=REPLAY_START_PLAY;
			replayMSG.messageDATA=replay_array_slot;
			printf( M_POS1_1 M_CLEAR_SCREEN "Replaying Slot %d\n\n",replay_array_slot);
			msg_send(qREPLAY,replayMSG);
			vTaskDelay((3/portTICK_RATE_MS));
			ignore_slot_key_release=0;
			slot_key_binary=0;
			}
		}
		return 0;
	}
	return 0;
}

void men_enter_stopped_mode(int *M_MENMODE,int *mode_changed){
	printf( M_POS1_1 M_CLEAR_SCREEN "Stopped\n\n");
	vTaskDelay(100);
	printf( M_POS1_1 M_CLEAR_SCREEN "Hit any key to\ncentre arm.\n");
	*mode_changed=1;
	*M_MENMODE=M_MENMODE_STOPPED;
	return;
}

void men_store_key_change(int key,int shifted,int state, portTickType *xLastStateChange,
	portTickType *xNewStateChange){

	static replay_storage_s stateChangeValue;
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
		stateChangeValue.rValue=shifted;
		stateChangeValue.delayTime=*xNewStateChange-*xLastStateChange;
		stateChangeValue.state=state;
		replayMSG.messageID=REPLAY_RECORD;
		replayMSG.messageDATA=(unsigned int)&stateChangeValue;
		msg_send(qREPLAY,replayMSG);
		vTaskDelay((5/portTICK_RATE_MS));

	}
}

int men_ik_control(int key){

	ik_cart_pos_s delta={0,0,0};

	switch (key){
	case M_KP_KEY_A1:
	case M_KP_KEY_B1:
		return 1;
	case M_KP_KEY_A2:
		delta.x_pos=(double)MANUAL_IK_DELTA;
		break;
	case M_KP_KEY_B2:
		delta.x_pos=(double)-MANUAL_IK_DELTA;
		break;
	case M_KP_KEY_A3:
		delta.y_pos=(double)MANUAL_IK_DELTA;
		break;
	case M_KP_KEY_B3:
		delta.y_pos=(double)-MANUAL_IK_DELTA;
		break;
	case M_KP_KEY_A4:
		delta.z_pos=(double)MANUAL_IK_DELTA;
		break;
	case M_KP_KEY_B4:
		delta.z_pos=(double)-MANUAL_IK_DELTA;
		break;
	default:
		return 0;
	}
	ik_move_delta(delta);
	//vTaskDelay(MS2TICKS(ik_move_delta(delta)));
	return 0;
}
