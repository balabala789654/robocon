#ifndef _MOTOR_FEEDBACK_H
#define _MOTOR_FEEDBACK_H

//#include "main.h"
#include "pid.h"
#include "sys.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t  all_ecd;
    int32_t  count;

    uint8_t temperate;
    int16_t last_ecd;
	float pid_set_speed;
} motor_measure_t;

typedef struct
{
	float target_rpm;
	float target_angle;
	
	PidType pid_param;
	PidType angle_pid_param;
	motor_measure_t feedback;
}motor;



void motor_driver_can1(uint16_t stid, int i1,int i2,int i3,int i4);
void motor_driver_can2(uint16_t stid, int i1,int i2,int i3,int i4);
void CAN1_RX0_IRQnHandler(void);
void CAN2_RX1_IRQnHandler(void);


#endif
