#ifndef __GPIO_H
#define __GPIO_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED1 PDout(7)	// DS0
#define LED2 PDout(4)	// DS0
#define LED3 PDout(3)	// DS0
#define LED4 PDout(2)	// DS0

void gpio_Init(void);//��ʼ��		 				    
#endif
