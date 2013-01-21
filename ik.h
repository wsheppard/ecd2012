/*
Inverse kinematics 
*/

#ifndef IK_H
#define IK_H

/*Joint and Servo max and min values*/
#define S_MAX_0 100000
#define S_MIN_0 50000
#define Q_MAX_0 (220*M_PI/180)
#define Q_MIN_0 0

#define S_MAX_1 100000 
#define S_MIN_1 50000
#define Q_MAX_1 (180*M_PI/180)
#define Q_MIN_1 0

#define S_MAX_2 100000
#define S_MIN_2 50000
#define Q_MAX_2 (-120*M_PI/180)
#define Q_MIN_2 (120*M_PI/180)

/* cartesian position of end effector*/
typedef struct {
	double x_pos;
	double y_pos;
	double z_pos;
}ik_cart_pos_s;



int ik_calc_IK(xQueueHandle qMOVE,ik_cart_pos_s);

#endif
