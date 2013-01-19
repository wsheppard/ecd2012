/*
Inverse kinematics 
*/

#ifndef IK_H
#define IK_H

/*Joint and Servo max and min values*/
#define S_MAX_0 100000
#define S_MIN_0 50000
#define Q_MAX_0 180
#define Q_MIN_0 0

#define S_MAX_1 100000 
#define S_MIN_1 50000
#define Q_MAX_1 180
#define Q_MIN_1 0

#define S_MAX_2 100000
#define S_MIN_2 50000
#define Q_MAX_2 180
#define Q_MIN_2 0

/* cartesian position of end effector*/
typedef struct {
	int x_pos;
	int y_pos;
	int z_pos;
}ik_cart_pos_s;

/* specific move message*/
typedef struct {
        unsigned int messageID;
        unsigned int messageDATA;
        unsigned int ikPLACE;
        unsigned int ikTIME = 0;
}ik_message_s;

void ik_calc_IK(ik_cart_pos_s);

#endif