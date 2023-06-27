#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "Remote_Control.h"
#include "can.h"
#include "pid.h"
#include "motor_feedback.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "FreeRTOSConfig.h"
#include "math.h"
#include "rc.h"
#include "chassis.h"
#include <string.h>
#include "delay.h"
#include "sys.h"
#include "rabbit.h"
#include "timer.h"
#include "shoot.h"
#include "clamping.h"

#define ON 1
#define OFF 0

#define FreeRtos ON

#define PI 3.14159

#endif

