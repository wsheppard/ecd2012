/******************************************************************************
*
*       File: manager.h
*       Language: C
*       AUTHOR: S. W. Sheppard
*       E-Mail: sheppard.will@gmail.com
*       https://github.com/wsheppard/ecd2012
*       
*
*       Description:  
*           Declarations for manager module.
*
*
*******************************************************************************/


#ifndef MANAGER_H
#define MANAGER_H

/* Create all required threads and start kick off */
int man_start(void);
void man_key_down(int key);
void man_key_up(int key);

#endif
