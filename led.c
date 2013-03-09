/* Module to interface with the leds */

#include "system.h"
#include "ECD.h"
#include "io.h"

static int LEDS=0;
static int *LEDS_BASE=&LEDS;

int setLED(unsigned int value){

	IOWR(LEDS_BASE, 0, value);

	return ECD_OK;

}
