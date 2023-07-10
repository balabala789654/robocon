#ifndef _P3_H
#define _P3_H

#include "stm32f4xx.h"                  // Device header
#include "stdlib.h"
#include "sys.h"

typedef struct 
{
	int ch[4];
	char L2;
	char L1;
	char LU;
	char LL;
	char LD;
	char LR;
	char SE;
	char ST;
	char RL;
	char RD;
	char RR;
	char RU;
	char R1;
	char R2;
	char R_key;
	char L_key;	
	float voltage;
}p3;
	
extern p3 p3_data;

void head_verify(uint8_t _data[13]);
uint8_t * verify_data_func(uint8_t _data[13]);
p3 p3_remote_output(uint8_t _data[13]);
void p3_init(u32 bound);
void USART6_IRQHandler(void);




#endif

