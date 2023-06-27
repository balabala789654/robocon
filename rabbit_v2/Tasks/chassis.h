#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor_feedback.h"
#include "Remote_Control.h"

typedef struct
{
	float vx;
	float vy;
	float chassis_yaw;
	int max_chassis_speed;
	int max_chassis_spin;
	double* chassis_targer_vx;
	double* chassis_targer_vy;
	double* chassos_targer_vz;
	motor chassis_motor[4];
}CHASSIS;



void remote_contorl_speed(RC_ctrl_t* RC, CHASSIS* _chassis);
void motor_speed_compute(CHASSIS* _chassis);
void address_transform(CHASSIS* _chassis, double* _remote_massge_vx, double* _remote_massge_vy, double* _remote_massge_vz);
void set_chassis_max_speed(CHASSIS* _chassis, int _speed_vx, int _speed_vy);
void chassis_task(void *pvParameters);

#endif


