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

static int ik_degree_to_servo(unsigned int *servoVal,double degree,int servoMax,int servoMin, double qMax, double qMin); /*convert degrees to a servo understood format*/
static int ik_servo_to_degree(double *degree,unsigned int servoVal,int servoMax,int servoMin, double qMax, double qMin); /*convert servo values to degrees*/


/* calculates joint angles from cartesian position
*/
int ik_calc_IK(xQueueHandle qMOVE ,ik_cart_pos_s position){
// Links Length
//unsigned int l1 = 8.5+9;
double l2 = 10;
double l3 = 12 ;
double d5 = 4.5;

//unsigned int current_s2,current_s3;
//double current_q2,current_q3;
double xc, yc,zc,q1,q21,q22,q31,q32;
msg_message_s msgMessage;


if(sqrt(pow(position.x_pos,2)+pow(position.y_pos,2)+pow(position.z_pos,2))>(l2+l3+d5)){/*check for valid input*/
    	//printf("Desired position outside of workspace.");
    return ECD_ERROR;
    }
                                                                //p0e = [x_in y_in z_in]';

//theta 1(rotation around base)
q1  = atan2(position.y_pos,position.x_pos);                     //atan2(p0e(2),p0e(1)); - (y,x)

//p0c = p0e - d5.*[cos(q1) sin(q1) 0]';

//treating the remaining part of the arm as planar...
xc = position.x_pos - d5*cos(q1);                               //xc = p0c(1,1);
yc = position.y_pos - d5*sin(q1);                               //yc = p0c(2,1);
zc = position.z_pos;                                            //zc = p0c(3,1);
printf("Wrist position: (%.3f,%.3f,%.3f)\n",xc,yc,zc);

// Geometric solution of q2 and q3

//set 1
//q31 = atan2(+sqrt(1-pow(((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)),2)),((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)));
//q21 = -atan2(((l3*sin(q31))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2))),+sqrt(1-pow((l3*sin(q31))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)),2)))+atan2(zc,sqrt(pow(xc,2)+pow(yc,2)));
//q41 = M_PI/2 -(q21+q31);



//set 2
q32 = atan2(-sqrt(1-pow((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3),2)),((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)));
q22 = -atan2(((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2))),+sqrt(1-pow((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)),2)))+atan2(zc,sqrt(pow(xc,2)+pow(yc,2)));

/* we need to check which of the two solutions is closer to the current configuration...
*/
	//pwm_get_pos(M_MOVE_SERVO2, &current_s2);
	//pwm_get_pos(M_MOVE_SERVO3, &current_s3);
	
	//if (fabs(current_q2-q21)<=fabs(current_q2-q22)){ //solution 1
	if(0){//solution 1 should always be with a negative angle for q2. The arm cant do this, so no need to compute...
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO1;
        ik_degree_to_servo(&msgMessage.sPLACE,q1,S_MAX_0,S_MIN_0, Q_MAX_0, Q_MIN_0);
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
                msgMessage.messageDATA = M_MOVE_SERVO2;
                ik_degree_to_servo(&msgMessage.sPLACE,q21,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO3;
                ik_degree_to_servo(&msgMessage.sPLACE,q31,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
		msg_send(qMOVE,msgMessage);
		
		
	}
	else {				//solution 2
		printf("Angles: q1 = %f, q2 = %f, q3 = %f\n",(q1*180/M_PI),(q22*180/M_PI),(q32*180/M_PI));
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO1;
        ik_degree_to_servo(&msgMessage.sPLACE,q1,S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0);
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO2;
        ik_degree_to_servo(&msgMessage.sPLACE,q22,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO3;
                ik_degree_to_servo(&msgMessage.sPLACE,q32,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
		msg_send(qMOVE,msgMessage);
		
		}

	
return ECD_OK;
}




static int ik_degree_to_servo(unsigned int *servoVal,double degree,int servoMax,int servoMin, double qMax, double qMin){
    *servoVal =(unsigned int)( degree*(((double)(servoMax - servoMin))/(qMax-qMin)) + (double)servoMax);
        if((*servoVal <=servoMax) || (*servoVal >= servoMin)){/*is the value within the servo range?*/
                return ECD_OK;
        }
        else{
                return ECD_ERROR;
        }
    }

static int ik_servo_to_degree(double *degree,unsigned int servoVal,int servoMax,int servoMin, double qMax, double qMin){
        if((*degree <=qMax) || (*degree >= qMin)){/*is the value within the angular range?*/
                return ECD_OK;
        }
        else{
                return ECD_ERROR;
        }
    }
