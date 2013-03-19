#ifndef MENU_H
#define MENU_H

#define M_MENMODE_STOPPED 0
#define M_MENMODE_CONTROL 1
#define M_MENMODE_RECORD_RT 2
#define M_MENMODE_REPLAY 3
#define M_MENMODE_RECORD_WP 4
#define M_MENMODE_CONTROL_IK 5
#define MANUAL_IK_DELTA 0.1

#define M_CLEAR_LINE "\x1b[k"
#define M_CLEAR_SCREEN "\x1b[2J"
#define M_POS1_1 "\x1b[1;1H"
#define M_POS2_1 "\x1b[2;1H"



/* Function provides necessary queues and arrays to menu */
int menu_init(xQueueHandle,xQueueHandle,xQueueHandle);
int men_check_menu(unsigned state, int key);


#endif
