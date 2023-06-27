#ifndef _RABBIT_H
#define _RABBIT_H

//#include "main.h"
#include "chassis.h"
#include "shoot.h"
#include "clamping.h"


typedef struct
{
	char rabbit_state;
	CHASSIS chassis;
	SHOOT shoot;
	CLAMPING clamping;
}robot;

extern robot rabbit;
void rabbit_init(void);
void FreeRtos_start(void);
void chssis_task_start(void);
void shoot_task_start(void);
void clamping_task_start(void);
void function_task_start(void);

#endif

