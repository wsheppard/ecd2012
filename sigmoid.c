/* Sigmoid module. */
#include <system.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include "ECD.h"
#include "sigmoid.h"
#include "task.h"
#include <io.h>

static xSemaphoreHandle xSemaphore = NULL;


/* Struct for euler stuff */
typedef struct {
	float input;
  	float output;
	unsigned int state;
  	unsigned int test;
  } euler_s;

typedef union{
	  	  float fl;
	  	  unsigned int ui;
	  	  signed int si;
	  	  void* ptr;
  }alltypes_u;

/* Tests basic sigmoid functionality */
int test_sigmoid(void){

	float M,time,result;

	alltypes_u input;

	M = 2.24;
	time = 3.8;
	result = 0;

	printf("Starting sigmoid test....\n");

//	printf("Sigmoid block test register returns [%X].\n", IORD(EULERBLOCK_0_BASE,3));

	printf("Floats have byte count of [%d].\n",(int)sizeof(float));

	input.fl = M;

//	IOWR(EULERBLOCK_0_BASE,0,input.ui);

	/* Check it's corrent */
//	input.ui = IORD(EULERBLOCK_0_BASE, 0);


//	printf("Inputs are M=[%4.2f], time=[%4.2f].\n",M,time);

	printf("IORD returns [%4.8f].\n", input.fl);

	sigmoid(M, time, &result);

	printf("Sigmoid returned [%4.8f].\n", result);

	return 0;
}

/* A normalized sigmoid - result is always 0-1
 * M is a time value which must be the same unit as the "time" parameter and represents
 * the half-time point*/
int sigmoid(float M, float time, float*result) {

	alltypes_u input,output;

	/* Semaphore is normally created by the movement init function.. */
	if(xSemaphore == NULL){
		xSemaphore = xSemaphoreCreateMutex();
	}

	/* The M value is the half time point where the gradient is maximum */

	/* Boundary checks ? */

	/* Do the sigmoid calcs */
	input.fl = (-1 * (SIGMOID_ERR) * (time - M)) / M;

	if (xSemaphore != NULL) {
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE) {

			/* Write to block and wait for the euler block to become ready -
			 * this is known at 17 cycles. So
			 * really we should just run 17nops. Remember a CS takes many more ! So let's
			 * give it the benefit of the doubt and just run nop if we can't use it. If all the
			 * other servos are trying to use it right at this moment then maybe we've got problems.
			 * We could make this a CRITICAL section...*/

			//IOWR(EULERBLOCK_0_BASE,0,input.ui);

			//while (IORD(EULERBLOCK_0_BASE,2)==0);

			//output.ui = IORD(EULERBLOCK_0_BASE, 1);

			xSemaphoreGive( xSemaphore);

		} else {
			printf("Servo couldn't get EULERBLOCK semaphore! \n");
			// We could not obtain the semaphore and can therefore not access
			// the shared resource safely.
		}

	}

	else{
		printf("ERROR: Sigmoid sempahore is NULL. Returning.\n");
		return ECD_ERROR;
		}

	//*result = 1.0/(1 + output.fl);
	*result=0.5;
	return ECD_OK;
}

