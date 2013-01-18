/*
 * ik.c
 *
 *  Created on: 12 Dec 2012
 *      Author: Raphael Nagel
 *
 *      This will hold the inverse kinematics stuff. soon probably
 *      This is the latest version
 */
 
 /* Standard includes. */
#include <stdio.h>
#include "math.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Local includes */
#include "messaging.h"
#include "messages.h"
#include "keypad.h"
#include "movement.h"
#include "pwm.h"
#include "display.h"





/* cartesian position of end effector*/
typedef struct {
	int x_pos;
	int y_pos;
	int z_pos;;
}ik_cart_pos_s;


/* calculates joint angles from cartesian position
*/
void ik_calc_IK(ik_cart_pos_s position){
// Links Length
unsigned int l1 = 8.5+9;
unsigned int l2 = 10;
unsigned int l3 = 12 ;
unsigned int d5 = 4.5;
int xc, yc,zc,q1,q2,q3,current_q2,current_q3;

																//p0e = [x_in y_in z_in]';

//theta 1(rotation around base)
q1  = atan2(position.y_pos,position.x_pos);			//atan2(p0e(2),p0e(1));

//p0c = p0e - d5.*[cos(q1) sin(q1) 0]';

//treating the remaining part of the arm as planar...
xc = position.x_pos - d5*cos(q1); 					//xc = p0c(1,1);
yc = position.y_pos - d5*sin(q1);						//yc = p0c(2,1);
zc = position.z_pos;										//zc = p0c(3,1);


// Geometric solution of q2 and q3

//set 1

q31 = atan2(+sqrt(1-((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3))^2),((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3)));
q21 = -atan2(((l3*sin(q31))/sqrt(xc^2+yc^2+zc^2)),+sqrt(1-((l3*sin(q31))/sqrt(xc^2+yc^2+zc^2))^2))+atan2(zc,sqrt(xc^2+yc^2));
//q41 = pi/2 -(q21+q31);



//set 2
q32 = atan2(-sqrt(1-((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3))^2),((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3)));
q22 = -atan2(((l3*sin(q32))/sqrt(xc^2+yc^2+zc^2)),+sqrt(1-((l3*sin(q32))/sqrt(xc^2+yc^2+zc^2))^2))+atan2(zc,sqrt(xc^2+yc^2));

/* we need to check which of the two solutions is closer to the current configuration...
*/
	pwm_get_pos(M_MOVE_SERVO2, &current_q2);
	pwm_get_pos(M_MOVE_SERVO3, &current_q3);
	
	if(abs(current_q2-q21)<=abs(current_q2-q22)){ //solution 1
		msgMessage.messageID = M_MOVE_IK;				
		msgMessage.messageDATA = M_MOVE_SERVO1 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);	
		
		msgMessage.messageID = M_MOVE_IK;								
		msgMessage.messageDATA = M_MOVE_SERVO2 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO3 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		
		
	}
	else {				//solution 2
		msgMessage.messageID = M_MOVE_IK;				
		msgMessage.messageDATA = M_MOVE_SERVO1 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);	
		
		msgMessage.messageID = M_MOVE_IK;								
		msgMessage.messageDATA = M_MOVE_SERVO2 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO3 | M_MOVE_DIRMASK;
		msg_send(qMOVE,msgMessage);		
		
		}

	

}




