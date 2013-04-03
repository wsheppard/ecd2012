/******************************************************************************
*
*       File: messages.h
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Defines representing the messages in the system.
*
*
*******************************************************************************/


/* All the different messages and what they mean */

#ifndef MESSAGES_H
#define MESSAGES_H


/* For the KEYPAD module */
#define M_KP_EVENT 1 /* A keypad change of state has occurred */

/* Keys for the KEYPAD module - each is represented by a bit in a 2byte variable */
#define M_KP_KEY_A1 (1 << 0)
#define M_KP_KEY_A2 (1 << 1)
#define M_KP_KEY_A3	(1 << 2)
#define M_KP_KEY_A4	(1 << 3)
#define M_KP_KEY_B1	(1 << 4)
#define M_KP_KEY_B2	(1 << 5)
#define M_KP_KEY_B3	(1 << 6)
#define M_KP_KEY_B4	(1 << 7)
#define M_KP_KEY_C1	(1 << 8)
#define M_KP_KEY_C2	(1 << 9)
#define M_KP_KEY_C3	(1 << 10)
#define M_KP_KEY_C4	(1 << 11)
#define M_KP_KEY_D1	(1 << 12)
#define M_KP_KEY_D2	(1 << 13)
#define M_KP_KEY_D3	(1 << 14)
#define M_KP_KEY_D4	(1 << 15)

/* Add extra messages here */

/* Movement module */
enum{
	M_MOVE_CONT, /* Start continous movement */
	M_MOVE_SPEC,  /* Move 'gracefully' to a specific place */
	M_MOVE_STOP,  /* STOP moving */
	M_MOVE_PLAYBACK,  /* Start the playback function */
	M_MOVE_SAVE,  /* Save current position */
	M_MOVE_RESET,  /* delete all saved positions */
	M_MOVE_RMLAST,  /* Remove last saved position */
	M_MOVE_IK,	/*move gracefully to the calculated IK position */
	M_COUNT
};

/* Some macros to help put the messaging together */
#define M_MOVE_SPEC_MESSAGE(SPEED, POSITION)	((unsigned)((((SPEED) & 0xFFFFFU)<<(3*8))|((POSITION)&0xFFFU)))
#define M_MOVE_SPEC_SPEED(MESSAGE)			(((MESSAGE)>>(3*8))&(0xFFFFFU))
#define M_MOVE_SPEC_POSITION(MESSAGE)		((MESSAGE)&(0xFFFU))


/* For the MOVE CONT message, the DATA is as follows:
	* BIT0-7 is the servo number
	* BIT8 is the direction
	*/
#define M_MOVE_SERVO1 0
#define M_MOVE_SERVO2 1
#define M_MOVE_SERVO3 2
#define M_MOVE_SERVO4 3


/* For the MOVE IK message the DATA is:
        *BIT0-n is the servo number
        
    there is also a PLACE and TIME:
        *PLACE is a u int of the desired servo position in servo parameters
        *TIME can hold the time allowed for a sigmoid move to the target position
        */
        
        
/* Masks for the servo messages */
#define M_MOVE_SERVOMASK 255 /* The servo number (4bits) */
#define M_MOVE_DIRMASK 256   /* THe direction bit (1bit) */
#define M_MOVE_PWMMASK_IK 0x01fffe0 /*The (pwm value-50000) shifted to the left by 5 and therefore at b5-b20*/
#define M_MOVE_SERVOMASK_IK 0x01E /*servo information at bit 1-b4*/
#define M_MOVE_SPECSPEEDMASK_IK 0x0ffe00000 /*time information for the duration of a movement at b21-b31*/

#define M_MOVE_PWMOFFSET_IK 5
#define M_MOVE_SPECSPEEDOFFSET_IK 21

#define M_IK_SERVO_MESSAGE(ID, GOAL, SPEED) 	(((ID<<1) & M_MOVE_SERVOMASK_IK) | (((GOAL - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK) | ((SPEED<<M_MOVE_SPECSPEEDOFFSET_IK) & M_MOVE_SPECSPEEDMASK_IK))
#define M_IK_SERVO_GOAL(DATA) 					(((DATA & M_MOVE_PWMMASK_IK)>>M_MOVE_PWMOFFSET_IK)+PWM_OFFSET)
#define M_IK_SERVO_SPEED(DATA) 						(((DATA & M_MOVE_SPECSPEEDMASK_IK)>>M_MOVE_SPECSPEEDOFFSET_IK)*MOVE_IK_MSG_COMPRESSION)


#endif
