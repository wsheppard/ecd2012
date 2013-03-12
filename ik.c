/*
 * ik.c
 *
 *  Created on: 12 Dec 2012
 *      Author: Raphael Nagel
 *
 *      This holds the  kinematics calculations.
 */

 
 /* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
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
static int ik_servo_to_rad(double *rad,unsigned int servoVal,int servoMax,int servoMin, double qMax, double qMin);

unsigned int data_temp;

const double  l2 = 10;
const double l3 = 13 ;
const double d5 = 6.7;
static xQueueHandle qMOVE;


int ik_init(xQueueHandle qMoveHandle){
	qMOVE=qMoveHandle;
	return ECD_OK;
}


/* calculates joint angles from Cartesian position
*/
int ik_calc_IK(ik_cart_pos_s position, msg_message_s *msgMessage0, msg_message_s *msgMessage1, msg_message_s *msgMessage2, float *move_time ){

	double xc, yc,zc,q1,q22,q32;
	int servoReturn = ECD_ERROR;
#if IK_DEBUG
	double tq1 = 0;
	double tq22 = 0;
	double tq32 = 0;
#endif

	/*The current pwm value of each servo*/
	unsigned int pos[4];
	unsigned int x;
	unsigned int goal[4];
	signed int longest_distance=0;
	float longest_time;
	unsigned int speed[4];
		
		
	if(sqrt(pow(position.x_pos,2)+pow(position.y_pos,2)+pow(position.z_pos,2))>(l2+l3+d5)){/*check for valid input*/
	//printf("Desired position outside of workspace.");
		return ECD_ERROR;
	}


	//theta 1(rotation around base)
	q1  = atan2(position.y_pos,position.x_pos);


	//Find the wrist position so that we can use these as our goal x,y,z coordinates
	xc = position.x_pos - d5*cos(q1);
	yc = position.y_pos - d5*sin(q1);
	zc = position.z_pos;
	//printf("Wrist position: (%.3f,%.3f,%.3f)\n",xc,yc,zc);

	/*Only solution 2 is used as solution 1 always produces a negative q2
	 * which isn't a possible configuration for this arm*/


	//set 2
	q32 = atan2(-sqrt(1-pow((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3),2)),((pow(xc,2)+pow(yc,2)+pow(zc,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3)));
	q22 = -atan2(((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2))),+sqrt(1-pow((l3*sin(q32))/sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)),2)))+atan2(zc,sqrt(pow(xc,2)+pow(yc,2)));
#if 0 //move all servos at the same speed
	//solution 2
	printf("Angles: q1 = %f, q2 = %f, q3 = %f\n",(q1*RAD2DEG),(q22*RAD2DEG),(q32*RAD2DEG));
	msgMessage.messageID = M_MOVE_IK;
	ik_rad_to_servo(&data_temp,q1,S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0);
	msgMessage.messageDATA = (((M_MOVE_SERVO4<<1) & M_MOVE_SERVOMASK_IK) | (((data_temp - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK)); /**/
	msg_send(qMOVE,msgMessage);


	msgMessage.messageID = M_MOVE_IK;
	ik_rad_to_servo(&data_temp,q22,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
	msgMessage.messageDATA = (((M_MOVE_SERVO3<<1) & M_MOVE_SERVOMASK_IK) | (((data_temp - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK));
	msg_send(qMOVE,msgMessage);


	msgMessage.messageID = M_MOVE_IK;
	ik_rad_to_servo(&data_temp,q32,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
	msgMessage.messageDATA = (((M_MOVE_SERVO2<<1) & M_MOVE_SERVOMASK_IK) | (((data_temp - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK));
	msg_send(qMOVE,msgMessage);

#else //change each servos movement speed so that the movement time is equal to the largest move - all servos start and finish at the same time
	printf("Angles: q1 = %f, q2 = %f, q3 = %f\n",(q1*RAD2DEG),(q22*RAD2DEG),(q32*RAD2DEG));

	if(ik_rad_to_servo(&goal[M_MOVE_SERVO4],q1,S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0) == ECD_OK){
		servoReturn = ECD_OK;
	}

#if IK_DEBUG //revert the servo values back to degrees and print them
	ik_servo_to_rad(&tq1,goal[M_MOVE_SERVO4],S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0);
	printf("tq1: %f\n",tq1*RAD2DEG);
#endif

	if(ik_rad_to_servo(&goal[M_MOVE_SERVO3],q22,S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1) == ECD_OK){
		servoReturn = ECD_OK;
	}
#if IK_DEBUG
	ik_servo_to_rad(&tq22,goal[M_MOVE_SERVO3],S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1);
	printf("tq22: %f\n",tq22*RAD2DEG);
#endif
	if(ik_rad_to_servo(&goal[M_MOVE_SERVO2],q32,S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2) == ECD_OK){
		servoReturn = ECD_OK;
	}
#if IK_DEBUG
	ik_servo_to_rad(&tq32,goal[M_MOVE_SERVO2],S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2);
	printf("tq32: %f\n",tq32*RAD2DEG);

	printf("servo values: s-q1: %d, s-q2: %d, s-q3: %d\n",goal[Q1],goal[Q2], goal[Q3]);
	printf("recalc Angles: q1 = %f, q2 = %f, q3 = %f\n",(tq1*RAD2DEG),(tq22*RAD2DEG),(tq32*RAD2DEG));
#endif

	for(x=1;x<PWM_COUNT;x++){ /*find the longest distance to move from all servos and calculate the time needed for that move*/
		pwm_get_pos(x,&pos[x]); /*get the currrent pwm position for servos all but the gripper*/
		if(abs((goal[x]-pos[x]))> longest_distance){
			longest_distance = abs(goal[x]-pos[x]);
		}
		longest_time = (float)longest_distance/MOVE_SPEC_STD_SPEED;
		*move_time = longest_time;
	}
	for(x=1;x<PWM_COUNT;x++){/*update the servo movement speeds, so that the time is the same on all of them*/
		speed[x] = (unsigned int)((abs(goal[x]-pos[x]) / longest_time) / MOVE_IK_MSG_COMPRESSION); /*M_MOVE_SPECSPEEDMASK_IK / MOVE_IK_MSG_COMPRESSION is < 2^11. Ergo it fits within the msgData*/
	}
	msgMessage0->messageID = M_MOVE_IK;
	//msgMessage.messageDATA = (((M_MOVE_SERVO4<<1) & M_MOVE_SERVOMASK_IK) | (((goal[M_MOVE_SERVO4] - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK) | ((speed[M_MOVE_SERVO4]<<M_MOVE_SPECSPEEDOFFSET_IK) & M_MOVE_SPECSPEEDMASK_IK)); /**/
	msgMessage0->messageDATA = M_IK_SERVO_MESSAGE(M_MOVE_SERVO4, goal[M_MOVE_SERVO4], speed[M_MOVE_SERVO4]);


	msgMessage1->messageID = M_MOVE_IK;
	//msgMessage.messageDATA = (((M_MOVE_SERVO3<<1) & M_MOVE_SERVOMASK_IK) | (((goal[M_MOVE_SERVO3] - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK) | ((speed[M_MOVE_SERVO3]<<M_MOVE_SPECSPEEDOFFSET_IK) & M_MOVE_SPECSPEEDMASK_IK));
	msgMessage1->messageDATA = M_IK_SERVO_MESSAGE(M_MOVE_SERVO3, goal[M_MOVE_SERVO3], speed[M_MOVE_SERVO3]);


	msgMessage2->messageID = M_MOVE_IK;
	//msgMessage.messageDATA = (((M_MOVE_SERVO2<<1) & M_MOVE_SERVOMASK_IK) | (((goal[M_MOVE_SERVO2] - PWM_OFFSET)<<M_MOVE_PWMOFFSET_IK) & M_MOVE_PWMMASK_IK) | ((speed[M_MOVE_SERVO2]<<M_MOVE_SPECSPEEDOFFSET_IK) & M_MOVE_SPECSPEEDMASK_IK));
	msgMessage2->messageDATA = M_IK_SERVO_MESSAGE(M_MOVE_SERVO2, goal[M_MOVE_SERVO2], speed[M_MOVE_SERVO2]);

#endif

	return servoReturn; /**/
}


int ik_calc_FK(ik_cart_pos_s *gripper_position){ /*reads the current pwm values and calculates the x, y and z coordinates*/
	int x;
	unsigned int pwm_val[4];

	double pos[4];

	for(x=1;x<PWM_COUNT;x++){
		pwm_get_pos(x,&pwm_val[x]); /*get the pwm positions for all servos but the gripper*/
								/*pos[3] = hip - q1 - M_MOVE_SERVO4*/
								/*pos[2] = shoulder - q2 - M_MOVE_SERVO3*/
								/*pos[1] = elbow - q3 - M_MOVE_SERVO2*/

	}

	ik_servo_to_rad(&pos[Q1],pwm_val[M_MOVE_SERVO4],S_MAX_0, S_MIN_0, Q_MAX_0, Q_MIN_0); /*convert pwm values to rad*/
	ik_servo_to_rad(&pos[Q2],pwm_val[M_MOVE_SERVO3],S_MAX_1, S_MIN_1, Q_MAX_1, Q_MIN_1); /*convert pwm values to rad*/
	ik_servo_to_rad(&pos[Q3],pwm_val[M_MOVE_SERVO2],S_MAX_2, S_MIN_2, Q_MAX_2, Q_MIN_2); /*convert pwm values to rad*/
	pos[Q4]= (M_PI/2) -(pos[Q3]+pos[Q2]);
								/*pos[0] = wrist - q4 - M_MOVE_SERVO1 - passive joint: always parallel to ground.*/
//printf("Current angles: q1 = %f, q2 = %f, q3 = %f, q4 = %f\n", pos[Q1]*RAD2DEG, pos[Q2]*RAD2DEG, pos[Q3]*RAD2DEG, pos[4]*RAD2DEG);

	/*Forward Kinematics - find the gripper position from the angular values.*/
	gripper_position->x_pos = cos(pos[Q1]) * sin(pos[Q2]+pos[Q3]+pos[Q4]) * d5 + cos(pos[Q1]) * cos(pos[Q2]+pos[Q3]) * l3 + cos(pos[Q1]) * cos(pos[Q2]) * l2 ;	/*xt = c1.*s234.*d5+c1.*c23.*l3+c1.*c2.*l2;*/
	gripper_position->y_pos = sin(pos[3]) * sin(pos[Q2]+pos[Q3]+pos[Q4]) * d5 + sin(pos[Q1]) * cos(pos[Q2]+pos[Q3]) * l3 + sin(pos[Q1]) * cos(pos[Q2]) * l2 ;	/*yt = s1.*s234.*d5+s1.*c23.*l3+s1.*c2.*l2;*/
	gripper_position->z_pos = -cos(pos[Q2]+pos[Q3]+pos[Q4]) * d5 + sin(pos[Q2]) * l2 + sin(pos[Q2]+pos[Q3]) * l3; 											/*zt = -c234.*d5+s2.*l2+s23.*l3;*/


	return ECD_OK;

}





static int ik_rad_to_servo(unsigned int *servoVal,double rad,int servoMax,int servoMin, double qMax, double qMin){

    *servoVal = (unsigned int)( (rad-qMin)*((double)(servoMax - servoMin)) / (qMax-qMin) + (double)servoMin);

    if((*servoVal <=servoMax) && (*servoVal >= servoMin)){/*is the value within the servo range?*/
                return ECD_OK;
        }
        else{
            if(*servoVal >servoMax){
                *servoVal = servoMax;
            }

            if(*servoVal <servoMin){
                *servoVal = servoMin;
            }
                return ECD_ERROR;
        }

    }

static int ik_servo_to_rad(double * rad,unsigned int servoVal,int servoMax, int servoMin, double qMax, double qMin){
    *rad = (double) ((((double)servoVal)-((double)servoMin)) * (qMax-qMin) / (((double)servoMax) - ((double)servoMin)) + qMin);

    if((*rad <= qMax) || (*rad >= qMin)){/*is the value within the servo range?*/
                return ECD_OK;
        }
        else{

                return ECD_ERROR;
        }

    }

unsigned int ik_move_goal(ik_cart_pos_s goal){/*tries to move the arm to a goal position regardless of it being within the workspace */
	msg_message_s msgMessage[3];
	int rVal = ECD_ERROR;
	float move_time = 0;
	rVal =  ik_calc_IK(goal, &msgMessage[0], &msgMessage[1], &msgMessage[2], &move_time); /*Calculate the values*/

	msg_send(qMOVE,msgMessage[0]);/*send them off to the servos*/
	msg_send(qMOVE,msgMessage[1]);
	msg_send(qMOVE,msgMessage[2]);

	return (unsigned int)(move_time * 1000);
}

int ik_move_delta(ik_cart_pos_s delta){/*moves the arm along the x, y and z axis if it is possible.*/
	ik_cart_pos_s current_position, next_position;
	msg_message_s msgMessage[3];
	int rVal = ECD_ERROR;
	float move_time = 0;


	ik_calc_FK(&current_position); /*find the current position*/

	next_position.x_pos = current_position.x_pos + delta.x_pos; /*calculate the next position to be moved to.*/
	next_position.y_pos = current_position.y_pos + delta.y_pos;
	next_position.z_pos = current_position.z_pos + delta.z_pos;

	/*calculate the pwm values needed to reach the next position and check if we are within the workspace boundaries of our arm*/
	rVal = ik_calc_IK(next_position, &msgMessage[0], &msgMessage[1], &msgMessage[2], &move_time);


	if(rVal == ECD_OK){/*if we can move to that position*/
		msg_send(qMOVE,msgMessage[0]);/*send the commands off to the servos*/
		msg_send(qMOVE,msgMessage[1]);
		msg_send(qMOVE,msgMessage[2]);

		return ECD_OK;
	}else{/*if we cannot move dont send the commands but print an error message*/
		printf("Cannot move to that position\n");

		return ECD_ERROR;
	}
}

