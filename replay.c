#include "messaging.h"
#include "replay.h"
#include "pwm.h"
#include "ik.h"
#include "manager.h"
#include "movement.h"
#include "math.h"
#include "menu.h"

static xQueueHandle qMOVE;
static xQueueHandle qREPLAY;
static xQueueHandle qKP;
static xQueueHandle qMENU;
static replay_storage_s replay_storage_array[NUM_REPLAY_SLOTS][NUM_REPLAY_STEPS]={0};

static void man_stop_all_pwm();
static void man_replay(void*params);

int replay_init(xQueueHandle qMoveHandle,xQueueHandle qRHandle,xQueueHandle qMenuHandle){

	qMOVE=qMoveHandle;

	qREPLAY=qRHandle;
	
	qMENU=qMenuHandle;
	
	xTaskCreate( man_replay, "ReplayTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	
	return ECD_OK;
}

/* Replay task that block waits on a queue till told to start. It then begins replaying and
performing non blocking queries on the queue monitoring for stop messages intermittently whilst
performing the replay operations*/
static void man_replay(void*params){
	int x=0,slot=0, num_delays=0;
	portTickType xLastWakeTime;
	replay_storage_s replay_mode;
	msg_message_s msgREPLAY, mMessage;
	unsigned int totaltime;

	for(;;){

		msg_recv_block(qREPLAY,&msgREPLAY);

		switch (msgREPLAY.messageID){
		case REPLAY_START_PLAY:
			/*First value in replay array defines what sort of replay this will be
			so initialise array iterator variable to next value. Initialise replay slot with the
			value recieved in the message */
			x=1;

			slot=msgREPLAY.messageDATA;

			/* if realtime replay slot */
			if ( replay_storage_array[slot][0].state ==REPLAY_RT ){
				/*Keep sending commands whilst there are still replay steps left,
				  not end of array and have not recieved stop message*/
				while((replay_storage_array[slot][x].state !=REPLAY_END) && 
					(x != (NUM_REPLAY_STEPS+1)) && (msgREPLAY.messageID!=REPLAY_STOP_PLAY)){
						/*Non blocking wait on Stop messages on replay queue*/
						msg_recv_noblock(qREPLAY,&msgREPLAY);
						/* A single delay that is the full length of the period between button presses could result
						in a larg(ish) period of time between pressing stop and the replay actually stopping. To overcome
						this I've split the delay into suitable sized chunks and make it check intermittently for stop messages. 
						It's either this or implementing some other way of directly stopping the pwms. E.G an interrupt */
						num_delays=replay_storage_array[slot][x].delayTime / STOP_POLL_DELAY;
						
						xLastWakeTime=xTaskGetTickCount();
						
						while(num_delays){
							//xQueueReceive( qREPLAY, &msgREPLAY, STOP_POLL_DELAY );
							vTaskDelay(STOP_POLL_DELAY);
							msg_recv_noblock(qREPLAY,&msgREPLAY);
							
							if (msgREPLAY.messageID==REPLAY_STOP_PLAY)
								goto replay_stop;
							
							num_delays--;
						
						}

						vTaskDelayUntil(&xLastWakeTime,replay_storage_array[slot][x].delayTime);
						
						if (replay_storage_array[slot][x].state == REPLAY_BUTTON_UP){
							man_key_up(replay_storage_array[slot][x].keyPressed);
						}
						
						else if (replay_storage_array[slot][x].state == REPLAY_BUTTON_DOWN){
							man_key_down(replay_storage_array[slot][x].keyPressed);
						}

						x++;
				}
			}
			/* if waypoint replay slot */
			else if ( replay_storage_array[slot][0].state ==REPLAY_WP ){
				while((replay_storage_array[slot][x].state !=REPLAY_END) && 
					(x != (NUM_REPLAY_STEPS+1)) && (msgREPLAY.messageID!=REPLAY_STOP_PLAY)){
						/*Non blocking wait on Stop messages on replay queue*/
						msg_recv_noblock(qREPLAY,&msgREPLAY);
						/* Use ik move function to move arm to saved waypoint */
						/* ik_move_goal returns time move will take in ms which is then converted to Ticks*/
						/*add extra 20 ticks to total time to wait to ensure arm is always fully settled before commencing next move*/
						totaltime=MS2TICKS(man_send_move(replay_storage_array[slot][x].servoValues))+30;
						//ik_move_goal(replay_storage_array[slot][x].ik_position))+30;
						
						num_delays=totaltime / STOP_POLL_DELAY;
						
						xLastWakeTime=xTaskGetTickCount();
						
						while(num_delays){
							/* block wait on queue for STOP_POLL_DELAY length of time */
							//xQueueReceive( qREPLAY, &msgREPLAY, STOP_POLL_DELAY );
							vTaskDelay(STOP_POLL_DELAY);
							msg_recv_noblock(qREPLAY,&msgREPLAY);
							
							if (msgREPLAY.messageID==REPLAY_STOP_PLAY)
								goto replay_stop;
							
							num_delays--;
						}
						
						vTaskDelayUntil(&xLastWakeTime,totaltime);
						
						/* Send gripper position stored in rValue Variable straight to servos */
						//pwm_set_pos(0, replay_storage_array[slot][x].rValue);
						//vTaskDelay(100);
						
						x++;
				}
			}

			replay_stop:
			
			if (replay_storage_array[slot][x].state ==REPLAY_END){
				printf( M_POS1_1 M_CLEAR_SCREEN "Replay finished\nSucessfully\n");
			}
			
			else if (msgREPLAY.messageID==REPLAY_STOP_PLAY){
				printf( M_POS1_1 M_CLEAR_SCREEN "Replay was\nstopped\n");
			}
			
			/* Send stop message to all pwms in case of forced stop message */
			man_stop_all_pwm();
			
			break;
		case REPLAY_START_RECORD:
			/* initialise chosen slot in as a realtime recording */
			replay_mode.state=REPLAY_RT;
			slot=msgREPLAY.messageDATA;
			replay_storage_array[slot][0]=replay_mode;
			x=1;
			break;
		case REPLAY_START_RECORD_WP:
			/* initialise chosen slot as a waypoint recording */
			replay_mode.state=REPLAY_WP;
			slot=msgREPLAY.messageDATA;
			replay_storage_array[slot][0]=replay_mode;
			x=1;
			break;
		case REPLAY_RECORD:
			if (x == NUM_REPLAY_STEPS){
				mMessage.messageID = REPLAY_ARRAY_OUT_OF_BOUNDS;
				msg_send(qMENU,mMessage);
				break;
			}

			replay_storage_array[slot][x]=*(replay_storage_s *)msgREPLAY.messageDATA;
			x+=1;			
			break;
		case REPLAY_STOP_RECORD:
			replay_storage_array[slot][x]=*(replay_storage_s *)msgREPLAY.messageDATA;
			break;
		default:
			break;
		}
	}
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

unsigned int man_send_move(unsigned int *nextPos){
	int x;
	unsigned int currentPos[4];
	unsigned int servoSpeed;
	unsigned int longest_distance=0;
	float longest_time;
	float move_time;
	msg_message_s servo_msg={M_MOVE_IK,0};

	for(x=0;x<PWM_COUNT;x++){ /*find the longest distance to move from all servos and calculate the time needed for that move*/
		pwm_get_pos(x,&currentPos[x]); /*get the currrent pwm position for servos all but the gripper*/
		if(abs((nextPos[x]-currentPos[x]))> longest_distance){
			longest_distance = abs(nextPos[x]-currentPos[x]);
		}
		longest_time = (float)longest_distance/MOVE_SPEC_STD_SPEED;
		move_time = longest_time;
	}
	for(x=0;x<PWM_COUNT;x++){/*update the servo movement speeds, so that the time is the same on all of them*/
		servoSpeed = (unsigned int)((abs(nextPos[x]-currentPos[x]) / longest_time) / MOVE_IK_MSG_COMPRESSION); /*M_MOVE_SPECSPEEDMASK_IK / MOVE_IK_MSG_COMPRESSION is < 2^11. Ergo it fits within the msgData*/
		servo_msg.messageDATA=M_IK_SERVO_MESSAGE(x,nextPos[x],servoSpeed);
		msg_send(qMOVE,servo_msg);
	}
 return (unsigned int)(move_time * 1000);
}