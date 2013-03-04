
#ifndef MANAGER_H
#define MANAGER_H

#define M_MANMODE_STOPPED 0
#define M_MANMODE_CONTROL 1
#define M_MANMODE_RECORD 2
#define M_MANMODE_REPLAY 3

#define NUM_REPLAY_SLOTS 10
#define NUM_REPLAY_STEPS 1000

typedef struct replay_storage_ss{
	unsigned int keyPressed;
	portTickType delayTime;
	unsigned int state;
}replay_storage_s;

enum {
	REPLAY_END,
	REPLAY_BUTTON_UP,
	REPLAY_BUTTON_DOWN
};

#define REPLAY_STOP 0
#define REPLAY_START 1
#define STOP_POLL_DELAY 50

/* Create all required threads and start kick off */
int man_start(void);




#endif