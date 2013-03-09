#ifndef REPLAY_H
#define REPLAY_H


#define NUM_REPLAY_SLOTS 10
#define NUM_REPLAY_STEPS 1000

enum {
	REPLAY_END,
	REPLAY_BUTTON_DOWN,
	REPLAY_BUTTON_UP,
};

enum {
 REPLAY_STOP_PLAY, 
 REPLAY_START_PLAY,
 REPLAY_STOP_RECORD,
 REPLAY_START_RECORD,
 REPLAY_RECORD,
 REPLAY_ARRAY_OUT_OF_BOUNDS,
};

#define STOP_POLL_DELAY 50

typedef struct replay_storage_ss{
	unsigned int keyPressed;
	portTickType delayTime;
	unsigned int state;
}replay_storage_s;


int replay_init(xQueueHandle,xQueueHandle,xQueueHandle);

#endif