#ifndef __GPIO_H
#define __GPIO_H

#include "sys.h"

//LED�˿ڶ���
#define LED1 PDout(2)	// DS0
#define LED2 PDout(3)	// DS0
#define LED3 PDout(4)	// DS0

#define Lifting PEout(15)
#define clamping PEout(14)
#define Push PEout(13)
void gpio_Init(void);//��ʼ��		 				    
#endif
