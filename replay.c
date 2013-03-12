#include "messaging.h"
#include "replay.h"
#include "pwm.h"
#include "manager.h"
#include "ik.h"
#include "math.h"

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

/* Replay task that block waits on a queue till told to start. It then begins replaying and performing non blocking queries on the queue monitoring for
stop messages intermittently whilst performing the replay operations*/
static void man_replay(void*params){
	int replay_array_position=0,replay_array_slot=0;
	portTickType xLastWakeTime;
	int num_delays;
	replay_storage_s replay_mode;
	msg_message_s msgREPLAY;
	msg_message_s mMessage;
	unsigned int totaltime;
	for(;;){
		msg_recv_block(qREPLAY,&msgREPLAY);
		switch (msgREPLAY.messageID){
		case REPLAY_START_PLAY:
			/*initiliase to start of replay array when starting a replay action*/
			replay_array_position=1;
			replay_array_slot=msgREPLAY.messageDATA;
			if ( replay_storage_array[replay_array_slot][0].state ==REPLAY_RT ){
				/*Keep sending commands whilst there are still replay steps left, not end of array and have not recieved stop message*/
				while((replay_storage_array[replay_array_slot][replay_array_position].state !=REPLAY_END) && 
					(replay_array_position != (NUM_REPLAY_STEPS+1)) && (msgREPLAY.messageID!=REPLAY_STOP_PLAY)){
						/*Non blocking wait on Stop messages on replay queue*/
						msg_recv_noblock(qREPLAY,&msgREPLAY);
						/* A single delay that is the full length of the period between button presses could result in a larg(ish) period of time between pressing stop
						and the replay actually stopping. To overcome this I've split the delay into suitable sized chunks and make it check intermittently for stop messages. 
						It's either this or implementing some other way of directly stopping the pwms. E.G an interrupt */
						num_delays=replay_storage_array[replay_array_slot][replay_array_position].delayTime / STOP_POLL_DELAY;
						xLastWakeTime=xTaskGetTickCount();
						while(num_delays){
							vTaskDelay(STOP_POLL_DELAY);
							msg_recv_noblock(qREPLAY,&msgREPLAY);
							if (msgREPLAY.messageID==REPLAY_STOP_PLAY)
								break;
							num_delays--;
						}
						if (msgREPLAY.messageID==REPLAY_STOP_PLAY)
							break;
						vTaskDelayUntil(&xLastWakeTime,replay_storage_array[replay_array_slot][replay_array_position].delayTime);
						if (replay_storage_array[replay_array_slot][replay_array_position].state == REPLAY_BUTTON_UP){
							man_key_up(replay_storage_array[replay_array_slot][replay_array_position].keyPressed);
						}
						else if (replay_storage_array[replay_array_slot][replay_array_position].state == REPLAY_BUTTON_DOWN){
							man_key_down(replay_storage_array[replay_array_slot][replay_array_position].keyPressed);
						}
						replay_array_position++;
				}
			}
			if ( replay_storage_array[replay_array_slot][0].state ==REPLAY_WP ){
				while((replay_storage_array[replay_array_slot][replay_array_position].state !=REPLAY_END) && 
					(replay_array_position != (NUM_REPLAY_STEPS+1)) && (msgREPLAY.messageID!=REPLAY_STOP_PLAY)){
						/*Non blocking wait on Stop messages on replay queue*/
						msg_recv_noblock(qREPLAY,&msgREPLAY);
						totaltime=MS2TICKS(ik_move_goal(replay_storage_array[replay_array_slot][replay_array_position].ik_position))+20;/*extra 20 ticks just in case*/
						num_delays=totaltime / STOP_POLL_DELAY;
						xLastWakeTime=xTaskGetTickCount();
						while(num_delays){
							vTaskDelay(STOP_POLL_DELAY);
							msg_recv_noblock(qREPLAY,&msgREPLAY);
							if (msgREPLAY.messageID==REPLAY_STOP_PLAY)
								break;
							num_delays--;
						}
						if (msgREPLAY.messageID==REPLAY_STOP_PLAY)
							break;
						vTaskDelayUntil(&xLastWakeTime,totaltime);
						pwm_set_pos(0, replay_storage_array[replay_array_slot][replay_array_position].keyPressed);
						vTaskDelay(10);
						replay_array_position++;
				}
			}
			
			if (replay_storage_array[replay_array_slot][replay_array_position].state ==REPLAY_END){
				printf("Replay finished\nSucessfully\n");
			}
			else if (msgREPLAY.messageID==REPLAY_STOP_PLAY){
				printf("Replay was\nstopped\n");
			}
			/* When end of saved replay is reached set msgReplay to stop so that it does not re-enter play loop*/
			msgREPLAY.messageID=REPLAY_STOP_PLAY;
			/* Send stop message to all pwms in case of forced stop message */
			man_stop_all_pwm();
			break;
		case REPLAY_START_RECORD:
			replay_mode.state=REPLAY_RT;
			replay_array_slot=msgREPLAY.messageDATA;
			replay_storage_array[replay_array_slot][0]=replay_mode;
			replay_array_position=1;
			break;
		case REPLAY_START_RECORD_WP:
			replay_mode.state=REPLAY_WP;
			replay_array_slot=msgREPLAY.messageDATA;
			replay_storage_array[replay_array_slot][0]=replay_mode;
			replay_array_position=1;
			break;
		case REPLAY_RECORD:
			replay_storage_array[replay_array_slot][replay_array_position]=*(replay_storage_s *)msgREPLAY.messageDATA;
			replay_array_position+=1;
			if (replay_array_position == NUM_REPLAY_STEPS){
				mMessage.messageID = REPLAY_ARRAY_OUT_OF_BOUNDS;
				msg_send(qMENU,mMessage);
			}
			break;
		case REPLAY_STOP_RECORD:
			replay_storage_array[replay_array_slot][replay_array_position]=*(replay_storage_s *)msgREPLAY.messageDATA;
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
