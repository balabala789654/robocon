#ifndef CLAMPING_H
#define CLAMPING_H

#include "motor_feedback.h"

typedef struct 
{
	char clamping_switch;
	double* clamping_height;
	double* clamping_pitch;
	double* clamping_door;	
	
	int max_door_motor_rpm;
	int max_lifting_motor_rpm;
	int max_pitch_motor_rpm;
	
	motor door_motor[2];
	motor lifting_motor[2];
	motor pitch_motor;
	
}CLAMPING;

void clamping_task(void *pvParameters);
void door_open_or_close(CLAMPING* _clamping, char state);
void clamping_address_transform(CLAMPING* _clamping, double* remote_ch_3, double* remote_ch_1, double* remote_ch_4);
void clamping_init(void);
void pitch_motor_init(CLAMPING* _clamping, float params[2][6]);
void lifting_motor_init(CLAMPING* _clamping, float params[2][6]);
void door_motor_init(CLAMPING* _clamping, float params[2][6]);
void set_clamping_motor_rpm(CLAMPING* _clamping, int _lifting, int _door, int _pitch);

void lifting_start(CLAMPING* _clamping, double* _height);
void door_start(CLAMPING* _clamping, double* _door);
void pitch_start(CLAMPING* _clamping, double* _pitch);



#endif
