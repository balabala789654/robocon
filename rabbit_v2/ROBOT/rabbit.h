#ifndef _RABBIT_H
#define _RABBIT_H

//#include "main.h"
#include "chassis.h"
#include "shoot.h"


typedef struct
{
	CHASSIS chassis;
	SHOOT shoot;
	
}robot;

extern robot rabbit;
void rabbit_init(void);
void FreeRtos_start(void);
void chssis_task_start(void);
void shoot_task_start(void);
void clamping_task_start(void);

#endif

