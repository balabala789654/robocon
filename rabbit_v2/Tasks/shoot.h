#ifndef SHOOT_H
#define SHOOT_H

#include "motor_feedback.h"

typedef struct
{
	int max_shoot_speed;
	float shoot_target_speed;
	float shoot_target_angle;
	float shoot_feedback_speed;
	float shoot_pitch;
	motor motor_2006[3];
}SHOOT;


void shoot_task(void *pvParameters);
void loop_loading(int _angle_L, int _angle_R, char flag);


#endif


