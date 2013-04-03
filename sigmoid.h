/******************************************************************************
*
*       File: sigmoid.h
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Declarations for sigmoid module.
*
*
*******************************************************************************/


/* Basic sigmoid call */
int sigmoid(float M, float time, float*result);

/* ms between steps  - too small isn't going to get scheduled */
#define MOVE_SIGMOID_LATENCY MOVE_LATENCY

#define SIGMOID_ERR	6.2126
