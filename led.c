/******************************************************************************
*
*       File: led.c
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Module with wrapper for LEDs on the DE0.
*
*
*******************************************************************************/

/* Module to interface with the leds */

#include "system.h"
#include "ECD.h"
#include "io.h"

int setLED(unsigned int value){

	IOWR(LEDS_BASE, 0, value);

	return ECD_OK;

}
