/*
 * timer.c
 *
 *  Created on: 21 Feb 2013
 *      Author: Will
 */
#include "sys/alt_irq.h"

#include "system.h"

#include "altera_avalon_timer.h"
#include "altera_avalon_timer_regs.h"
#include "ECD.h"
#include "FreeRTOS.h"
#include <io.h>
#include <stdio.h>

volatile static int counter = 0;

void timerISR(void* context);

int timer_init(void){

	int ret;

	printf("Registering timer ISR...\n");

	ret = alt_ic_isr_register(
			PWM_TIMER_IRQ_INTERRUPT_CONTROLLER_ID,
			PWM_TIMER_IRQ,
			timerISR,
			NULL, NULL );

	IOWR_ALTERA_AVALON_TIMER_CONTROL(PWM_TIMER_BASE,1);

	printf("Register returned %d.\n", ret);

	printf("Timer regs are:\n %8X\n%8X\n%8X\n%8X\n.",
			IORD(PWM_TIMER_BASE,0),
			IORD(PWM_TIMER_BASE,1),
			IORD(PWM_TIMER_BASE,2),
			IORD(PWM_TIMER_BASE,3));

	usleep(1000000);

	printf("The counter is now: %d.\n", counter);

	return ECD_OK;
}


void timerISR(void* context){

	counter++;

	IOWR_ALTERA_AVALON_TIMER_STATUS(PWM_TIMER_BASE,0);
}
