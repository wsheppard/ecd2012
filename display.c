/******************************************************************************
*
*       File: display.c
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Display module - looks after displaying info to the user.
*
*
*******************************************************************************/

/* Local Includes */
#include "display.h"
#include "pwm.h"
#include "movement.h"
#include "led.h"

/* System Includes */
#include <stdio.h>
#include <system.h>



static void display_main(void*params);




int display_init(void){
	
	/* Create main task; return -1 on error */
	if (xTaskCreate( display_main, 
		(const signed char*)"Display Thread",
		configMINIMAL_STACK_SIZE, 
		NULL, 
		DISPLAY_PRIORITY, 
		NULL) != pdPASS){
	
			return ECD_ERROR;
	} 
	return ECD_OK;
}


void display_main(void*params){

	unsigned int* p_pwm;
	unsigned int pwm_value = 50000;
	unsigned int pos[4];
	int state[4];
	int x, led_counter = 0 ;

	unsigned int pwm_add = 2000;

	unsigned int *p_keypad_data;
	unsigned int *p_euler_data;

//	euler_s* p_euler;

	int xDelay = 1000 / portTICK_RATE_MS;

//	p_pwm = (unsigned int*)PWM_COMPONENT_0_BASE;

	p_keypad_data = (unsigned int *)KEYPAD_COMPONENT_0_BASE;

	for(;;){

#if 0
		fprintf(stderr,"EULER: In[%4.4f] State[%d] Out[%4.4f]\n",
					p_euler->input,
					p_euler->state,
					p_euler->output);


		p_euler->input = -2.25;
#endif

//		fprintf(stderr,"KEYPAD: %X\n",*(p_keypad_data+2));


#if 1
		for(x=0;x<PWM_COUNT;x++){ 
			pwm_get_pos(x,&pos[x]);
			pos[x]/=1000;
			move_get_state(x,&state[x]);
		}



		fprintf(stderr,"[%u:%u],[%u:%u],[%u:%u],[%u:%u].\n",
			pos[0],state[0],
			pos[1],state[1],
			pos[2],state[2],
			pos[3],state[3]);
		fflush( stdout );
#endif

#if 0
		fprintf(stderr,"PWM AT %u.\n",
					pwm_value/2000);
#endif

		setLED(led_counter++);
		//*(int*)pLEDS = led_counter++;

#if 0
		pwm_value += pwm_add;

		if(pwm_value >= 100000)
			pwm_add*=-1;

		if((pwm_value)<50000)
			pwm_add*=-1;


		*p_pwm = pwm_value;
		*(p_pwm+1) = pwm_value;
		*(p_pwm+2) = pwm_value;
		*(p_pwm+3) = pwm_value;
#endif
		vTaskDelay(xDelay);
	
	}


}



