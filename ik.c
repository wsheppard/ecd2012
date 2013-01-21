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

static int ik_rad_to_servo(unsigned int *servoVal,double rad,int servoMax,int servoMin, double qMax, double qMin); /*convert rads to a servo understood format*/


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
double xc, yc,zc,q1,q22,q32;
msg_message_s msgMessage;


if(sqrt(pow(position.x_pos,2)+pow(position.y_pos,2)+pow(position.z_pos,2))>(l2+l3+d5)){/*check for valid input*/
    	//printf("Desired position outside of workspace.");
    return ECD_ERROR;
    }
                                                                //p0e = [x_in y_in z_in]';

//theta 1(rotation around base)
q1  = atan2(position.y_pos,position.x_pos);                     //atan2(p0e(2),p0e(1)); - (y,x)


//treating the remaining part of the arm as planar...
xc = position.x_pos - d5*cos(q1);                               //xc = p0c(1,1);
yc = position.y_pos - d5*sin(q1);                               //yc = p0c(2,1);
zc = position.z_pos;                                            //zc = p0c(3,1);
printf("Wrist position: (%.3f,%.3f,%.3f)\n",xc,yc,zc);

/*Only solution 2 is used as solution 1 always produces a negative q2
 * which isn't a possible configuration for this arm*/


//set 2
q32 = atan2(-sqrt(1-pow((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3),2)),((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)));
q22 = -atan2(((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2))),+sqrt(1-pow((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)),2)))+atan2(zc,sqrt(pow(xc,2)+pow(yc,2)));

		//solution 2
		printf("Angles: q1 = %f, q2 = %f, q3 = %f\n",(q1*180/M_PI),(q22*180/M_PI),(q32*180/M_PI));
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO1;
        ik_rad_to_servo(&msgMessage.sPLACE,q1,S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0);
		msgMessage.sTIME = 2;
		printf("Servo 1 PWM value: %d \n",msgMessage.sPLACE);
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO2;
        ik_rad_to_servo(&msgMessage.sPLACE,q22,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
		msgMessage.sTIME = 2;
		printf("Servo 2 PWM value: %d \n",msgMessage.sPLACE);
		msg_send(qMOVE,msgMessage);
		
		msgMessage.messageID = M_MOVE_IK;
		msgMessage.messageDATA = M_MOVE_SERVO3;
        ik_rad_to_servo(&msgMessage.sPLACE,q32,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
		msgMessage.sTIME = 2;
		printf("Servo 3 PWM value: %d \n",msgMessage.sPLACE);
		msg_send(qMOVE,msgMessage);
		

	
return ECD_OK;
}




static int ik_rad_to_servo(unsigned int *servoVal,double rad,int servoMax,int servoMin, double qMax, double qMin){
    *servoVal =(unsigned int)( rad*(((double)(servoMax - servoMin))/(qMax-qMin)) + (double)servoMin);
        if((*servoVal <=servoMax) || (*servoVal >= servoMin)){/*is the value within the servo range?*/
                return ECD_OK;
        }
        else{
                return ECD_ERROR;
        }
    }

