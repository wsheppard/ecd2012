/******************************************************************************
*
*       File: ECD.h
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Contains the project-wide defines / variables etc..
*
*
*******************************************************************************/

/* 
* 
*/
#ifndef ECD_H
#define ECD_H

#include <FreeRTOS.h>

/* Return values for this project */
#define ECD_OK 0
#define ECD_ERROR -1
#define ECD_NOMSG -2

/* This is for casting strings to pass to
 * FreeRTOS because it wants signed chars */
typedef const signed char * fStr;

#define TICKS_PER_MS (configTICK_RATE_HZ / 1000)
#define MS_PER_TICK (portTICK_RATE_MS)
#define MS2TICKS(A) ((TICKS_PER_MS) * A)
#define TICKS2MS(A) ((MS_PER_TICK) * A)

//#define lcd_printf(...) fprintf(lcd, __VA_ARGS__)
//#define jtag_printf(...) fprintf(jtag, __VA_ARGS__)

#endif

