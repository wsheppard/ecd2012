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
#include "ik.h"

static int ik_degree_to_servo(unsigned int *servoVal,unsigned int degree,int servoMax,int servoMin, int qMax, int qMin); /*convert degrees to a servo understood format*/


/* calculates joint angles from cartesian position
*/
int ik_calc_IK(xQueueHandle qMOVE ,ik_cart_pos_s position){
// Links Length
unsigned int l1 = 8.5+9;
unsigned int l2 = 10;
unsigned int l3 = 12 ;
unsigned int d5 = 4.5;
unsigned int current_q2,current_q3;
int xc, yc,zc,q1,q21,q22,q31,q32;

ik_message_s ikMessage;

if(sqrt((position.x_pos^(2))+(position.y_pos^(2))+(position.z_pos^(2)))>(l2+l3+d5)){/*check for valud input*/
    	//printf("Desired position outside of workspace.");
    return ECD_ERROR;
    }
                                                                //p0e = [x_in y_in z_in]';

//theta 1(rotation around base)
q1  = atan2(position.y_pos,position.x_pos);                     //atan2(p0e(2),p0e(1));

//p0c = p0e - d5.*[cos(q1) sin(q1) 0]';

//treating the remaining part of the arm as planar...
xc = position.x_pos - d5*cos(q1);                               //xc = p0c(1,1);
yc = position.y_pos - d5*sin(q1);                               //yc = p0c(2,1);
zc = position.z_pos;                                            //zc = p0c(3,1);


// Geometric solution of q2 and q3

//set 1
q31 = atan2(+sqrt(1-pow(((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)),2)),((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)));
q21 = -atan2(((l3*sin(q31))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2))),+sqrt(1-pow((l3*sin(q31))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)),2)))+atan2(zc,sqrt(pow(xc,2)+pow(yc,2)));
//q41 = pi/2 -(q21+q31);



//set 2
q32 = atan2(-sqrt(1-pow((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3),2)),((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)));
q22 = -atan2(((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2))),+sqrt(1-pow((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)),2)))+atan2(zc,sqrt(pow(xc,2)+pow(yc,2)));

/* we need to check which of the two solutions is closer to the current configuration...
*/
	pwm_get_pos(M_MOVE_SERVO2, &current_q2);
	pwm_get_pos(M_MOVE_SERVO3, &current_q3);
	
	if (fabs(current_q2-q21)<=fabs(current_q2-q22)){ //solution 1
		ikMessage.messageID = M_MOVE_IK;				
		ikMessage.messageDATA = M_MOVE_SERVO1;
        ik_degree_to_servo(&ikMessage.ikPLACE,q1,S_MAX_0,S_MIN_0, Q_MAX_0, Q_MIN_0);
		msg_ik_send(qMOVE,ikMessage);
		
		ikMessage.messageID = M_MOVE_IK;								
                ikMessage.messageDATA = M_MOVE_SERVO2;
                ik_degree_to_servo(&ikMessage.ikPLACE,q21,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
		msg_ik_send(qMOVE,ikMessage);
		
		ikMessage.messageID = M_MOVE_IK;
		ikMessage.messageDATA = M_MOVE_SERVO3;
                ik_degree_to_servo(&ikMessage.ikPLACE,q31,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
		msg_ik_send(qMOVE,ikMessage);
		
		
	}
	else {				//solution 2
		ikMessage.messageID = M_MOVE_IK;				
		ikMessage.messageDATA = M_MOVE_SERVO1;
                ik_degree_to_servo(&ikMessage.ikPLACE,q1,S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0);
		msg_ik_send(qMOVE,ikMessage);
		
		ikMessage.messageID = M_MOVE_IK;								
		ikMessage.messageDATA = M_MOVE_SERVO2;
                ik_degree_to_servo(&ikMessage.ikPLACE,q22,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
		msg_ik_send(qMOVE,ikMessage);
		
		ikMessage.messageID = M_MOVE_IK;
		ikMessage.messageDATA = M_MOVE_SERVO3;
                ik_degree_to_servo(&ikMessage.ikPLACE,q32,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
		msg_ik_send(qMOVE,ikMessage);
		
		}

	
return ECD_OK;
}




static int ik_degree_to_servo(unsigned int *servoVal,unsigned int degree,int servoMax,int servoMin, int qMax, int qMin){
    *servoVal = degree*((servoMax - servoMin)/(qMax-qMin)) + servoMax;
        if((*servoVal <=servoMax) || (*servoVal >= servoMin)){/*is the value within the servo range?*/
                return ECD_OK;
        }
        else{
                return ECD_ERROR;
        }
    }
