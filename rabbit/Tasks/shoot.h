#ifndef SHOOT_H
#define SHOOT_H

#include "motor_feedback.h"

typedef struct
{
	char shoot_switch;
	
	double* address_shoot_friction_rpm;
	double* address_shoot_motor_pitch;
	double* address_shoot_motor_enter;
	int max_friction_speed;
	int max_enter_speed;
	int max_pitch_speed;
	
	int shoot_friction_rpm;
	float shoot_target_angle;
	float shoot_feedback_speed;
	float shoot_pitch;
	
	motor enter_motor;
	motor shoot_pitch_motor;
}SHOOT;


void shoot_task(void *pvParameters);
void shoot_address_transform(SHOOT* _shoot, double* remote_ch_4, double* remote_ch_1, double* remote_ch_3);
void set_shoot_max_speed(SHOOT* _shoot, int _speed, int _enter_speed, int _pitch_speed);
void enter_motor_init(SHOOT* _shoot, float params[2][6]);
void shoot_pitch_motor_init(SHOOT* _shoot, float params[2][6]);

void friction_start(SHOOT* _shoot, double* _speed);
void enter_start(SHOOT* _shoot, double* remote_ch_1);
void shoot_pitch_start(SHOOT* _shoot, double* remote_ch_3);


#endif


