/*
 *
 *
 *
 *    Code for PWM interaction
 *
 */

#include "pwm.h"
#include "semphr.h"
#include <stdio.h>
#include "system.h"
#include <io.h>

/* Fill private data about servos, the position data is kept track of privately
   in this module - so might need some functions to get it out? */
int dummypwmadd[PWM_COUNT];
static pwm_servo_data_s servo_data[PWM_COUNT]={
	{(void*)&dummypwmadd[0],0,75000},
	{(void*)&dummypwmadd[1],0,75000},
	{(void*)&dummypwmadd[2],0,75000},
	{(void*)&dummypwmadd[3],0,75000},
};

static xSemaphoreHandle xSemaphore = NULL;

int pwm_init(void){
	
	int x;

	xSemaphore = xSemaphoreCreateMutex();

	for (x=0;x<PWM_COUNT;x++){
		pwm_set_pos(x,servo_data[x].defaultposition);
	}

	return ECD_OK;
}

int pwm_jump(int servo, int jump){
	
	unsigned int position;

	/* Get current servo position */
	pwm_get_pos(servo, &position);

	if ( (jump<0)&&((-jump)>(int)position)){
		/* Add "jump" to it */
		jump=(-(int)position);
	}

	position += jump;

	if ( (position<50000) ){
		position =50000;
	}

	if ( (position>100000) ){
			position =100000;
		}

	/* Write out new servo position */
	pwm_set_pos(servo, position);

	return ECD_OK;
}

int pwm_set_pos(int servo, unsigned int position){

	static unsigned int last_error = 0;

	/* Boundary check */
	if (  ((position>100000) || (position<50000)) && (position!=last_error)   ){
		fprintf(stderr,"Servo [%d] bounary error! Value: %d.\n",servo, position);
		last_error = position;
		return ECD_ERROR;
	}
	/* 	Write to hardware using HAL provided functions.
		these prevent data caching by the NIOS */
	IOWR(servo_data[servo].address,0,position);


	/* Now update the global variable which holds the PWM position data */
	 if( xSemaphore != NULL )
    {
        // See if we can obtain the semaphore.  If the semaphore is not available
        // wait 10 ticks to see if it becomes free.	
        if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.

            // ...

			servo_data[servo].position = position;

            // We have finished accessing the shared resource.  Release the 
            // semaphore.
            xSemaphoreGive( xSemaphore );
        }
        else
        {
			fprintf(stderr,"PWM_SET Couldn't get semaphore...\n");
            // We could not obtain the semaphore and can therefore not access
            // the shared resource safely.
        }
    }

	return ECD_OK;
}

int pwm_get_pos(int servo, unsigned int* position){

		 if( xSemaphore != NULL )
    {
        // See if we can obtain the semaphore.  If the semaphore is not available
        // wait 10 ticks to see if it becomes free.	
        if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.

            // ...

			*position = servo_data[servo].position;

            // We have finished accessing the shared resource.  Release the 
            // semaphore.
            xSemaphoreGive( xSemaphore );
        }
        else
        {
			fprintf(stderr,"Couldn't get semaphore...\n");
            // We could not obtain the semaphore and can therefore not access
            // the shared resource safely.
        }
    }


	return ECD_OK;
}	

