#ifndef MENU_H
#define MENU_H

#define M_MENMODE_STOPPED 0
#define M_MENMODE_CONTROL 1
#define M_MENMODE_RECORD_RT 2
#define M_MENMODE_REPLAY 3
#define M_MENMODE_RECORD_WP 4
#define M_MENMODE_CONTROL_IK 5
#define MANUAL_IK_DELTA 0.5

/* Function provides necessary queues and arrays to menu */
int menu_init(xQueueHandle,xQueueHandle,xQueueHandle);
int men_check_menu(unsigned state, int key);


#endif