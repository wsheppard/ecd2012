/* 
* Contains the project-wide defines / variables etc..
*/
#ifndef ECD_H
#define ECD_H

/* Return values for this project */
#define ECD_OK 0
#define ECD_ERROR -1
#define ECD_NOMSG -2

/* This is for casting strings to pass to
 * FreeRTOS becuase it wants signed chars */
typedef const signed char * fStr;

#endif

