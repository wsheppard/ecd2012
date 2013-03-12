/*
Inverse kinematics 
*/

#ifndef IK_H
#define IK_H

/*Joint and Servo max and min values*/
#define S_MAX_0 100000
#define S_MIN_0 50000
#define Q_MAX_0 (45*M_PI/180)
#define Q_MIN_0 (-45*M_PI/180)

#define S_MAX_1 100000 
#define S_MIN_1 50000
#define Q_MAX_1 (-45*M_PI/180)
#define Q_MIN_1 (45*M_PI/180)

#define S_MAX_2 100000
#define S_MIN_2 50000
#define Q_MAX_2 (-35*M_PI/180)
#define Q_MIN_2 (25*M_PI/180)

#define Q1 3
#define Q2 2
#define Q3 1
#define Q4 0

#define RAD2DEG 180/M_PI



/* cartesian position of end effector*/
typedef struct {
	double x_pos;
	double y_pos;
	double z_pos;
}ik_cart_pos_s;



int ik_calc_IK(ik_cart_pos_s position, msg_message_s *msgMessage0, msg_message_s *msgMessage1, msg_message_s *msgMessage2, float *move_time);
int ik_calc_FK(ik_cart_pos_s*);
unsigned int ik_move_goal( ik_cart_pos_s goal);
int ik_move_delta(ik_cart_pos_s delta);
int ik_init(xQueueHandle qMOVE);
#endif