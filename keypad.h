/******************************************************************************
*
*       File: keypad.h
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Declarations for keypad module.
*
*
*******************************************************************************/



#ifndef KEYPAD_H
#define KEYPAD_H

/* Free RTOS includes */
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Local Includes */
#include "ECD.h"
#include "messaging.h"

/*How much settling time there is for a button
KP_DELAY * DEBOUNCE_MS = settling time in ms .*/
#define DEBOUNCE_MS 1

/* How often to POLL (in ms between polls).*/
#define KP_DELAY (20/portTICK_RATE_MS) 



/* Task priority */
#define KP_PRIORITY tskIDLE_PRIORITY

/* Memory base address for the keypad module. */
#define KP_BASE_ADDRESS KEYPAD_COMPONENT_0_BASE

/* Called to start off the task; Give the message queue handle to 
	which it should put keypad events onto. if successful it will return ok. */
int kp_startTask(xQueueHandle);

#endif
