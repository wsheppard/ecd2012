/* Sigmoid module. */





/* Tests basic sigmoid functionality */
int test_sigmoid(void){


	float M,time,result;
	volatile euler_s* p_euler;


	p_euler = (euler_s*) EULERBLOCK_0_BASE;

	/* Semaphore is normally created by the movement init function.. */
	xSemaphore = xSemaphoreCreateMutex();

	M = 2.24;
	time = 2.24;
	result = 0;

	/* Do initial write to the first place. */
	p_euler->input = M;

	/* Check it's corrent */


	printf("Starting sigmoid test....\n");
	printf("Sigmoid block test register returns [%X].\n", p_euler->test);
	printf("Floats have byte count of [%d].\n",sizeof(float));
	printf("Sigmoid returns for [%4.2f] ---> [%4.2f].\n",M,p_euler->input);

	printf("Inputs are M=[%4.2f], time=[%4.2f].\n",M,time);

	usleep(3000);

	sigmoid(M, time, &result);

	printf("Sigmoid returned [%4.2f].\n", result);

	return 0;
}

/* A normalized sigmoid - result is always 0-1
 * M is a time value which must be the same unit as the "time" parameter and represents
 * the half-time point*/
int sigmoid(float M, float time, float*result) {

	volatile euler_s* p_euler;
	float fInput;

	/* The M value is the half time point where the gradient is maximum */

	/* Boundary checks ? */

	/* Do the sigmoid calcs */
	p_euler = (euler_s*) EULERBLOCK_0_BASE;

	fInput = (-1 * (SIGMOID_ERR) * (time - M)) / M;

	if (xSemaphore != NULL) {
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if (xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE) {

			p_euler->input = fInput;

			/* Wait for the euler block to become ready - this might hang!!!*/
					while (!p_euler->state){
						vTaskDelay(1);/*definitely 0, remove if redundant?*/
						//__asm("nop");
					}


			xSemaphoreGive( xSemaphore);
		} else {
			printf("Servo couldn't get EULERBLOCK semaphore! \n");
			// We could not obtain the semaphore and can therefore not access
			// the shared resource safely.
		}
	}

	*result = 1.0/(1+p_euler->output);

	return ECD_OK;
}
