#ifndef CLAMPING_H
#define CLAMPING_H


#include "motor_feedback.h"

typedef struct 
{
	float a;
}CLAMPING;

void clamping_task(void *pvParameters);

#endif

