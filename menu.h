#ifndef MENU_H
#define MENU_H

#define M_MANMODE_STOPPED 0
#define M_MANMODE_CONTROL 1
#define M_MANMODE_RECORD 2
#define M_MANMODE_REPLAY 3

/* Function provides necessary queues and arrays to menu */
int menu_init(xQueueHandle,xQueueHandle,xQueueHandle);
int man_check_menu(unsigned state, int key);


#endif