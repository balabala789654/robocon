#ifndef _MOTOR_FEEDBACK_H
#define _MOTOR_FEEDBACK_H

#include "pid.h"
#include "sys.h"


#define get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;											\
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                     \
}


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
	
	int initial_ecd;
	PidType pid_param_speed;
	PidType pid_param_angle;
	motor_measure_t feedback;
}MOTOR;



void motor_driver_can1(uint16_t stid, int i1,int i2,int i3,int i4);
void motor_driver_can2(uint16_t stid, int i1,int i2,int i3,int i4);
void CAN1_RX0_IRQnHandler(void);
void CAN2_RX1_IRQnHandler(void);
void motor_speed_pid_cal(MOTOR *_motor);
void motor_close(MOTOR *_motor);


#endif
