#include "p3.h"
#include "stm32f4xx.h"                  // Device header

static uint8_t head_1;
static uint8_t head_2;

uint8_t p3_receive[13];
uint8_t p3_verify_data[13];

p3 p3_data;

p3 p3_remote_output(uint8_t _data[13])
{
	p3 _p3;
	_p3.ch[3]=127-_data[6];
	_p3.ch[2]=127-_data[5];
	_p3.ch[1]=127-_data[8];
	_p3.ch[0]=127-_data[7];
	
	_p3.L2 = _data[9]>>7;
	_p3.L1 = (_data[9]&0x40)>>6;
	_p3.LU = (_data[9]&0x20)>>5;
	_p3.LL = (_data[9]&0x10)>>4;
	
	_p3.LD = (_data[9]&0x8)>>3;
	_p3.LR = (_data[9]&0x4)>>2;
	_p3.SE = (_data[9]&0x2)>>1;
	_p3.ST = (_data[9]&0x00)>>0;
	
	_p3.RL = 	_data[10]>>7;
	_p3.RD = 	(_data[10]&0x40)>>6;
	_p3.RR = 	(_data[10]&0x20)>>5;
	_p3.RU = 	(_data[10]&0x10)>>4;
	            
	_p3.R1 = 	(_data[10]&0x8)>>3;
	_p3.R2 = 	(_data[10]&0x4)>>2;
	_p3.R_key = (_data[10]&0x2)>>1;
	_p3.L_key = (_data[10]&0x01)>>0;

	_p3.voltage = (float)(_data[4]+200.0f)/100.0f;
	return _p3;
}

uint8_t * verify_data_func(uint8_t _data[13])
{	
	
	static uint8_t _verify[13];
	
	for(int i=head_1, j=0; j<13; j++)
	{
		_verify[j]=_data[i];
		i++;
		if(i==13) i=0;
		
	}
	return _verify;
		
}

void head_verify(uint8_t _data[13])
{
	for(int i=0; i<13; i++)
	{
		if(_data[i]==0xAA)
		{
			head_1=i;
			if(_data[i+1]==0xAA)
			{				
				head_2=i+1;
				break;
			}
			else if(_data[12-i]==0xAA)
			{
				head_2=12-i;
			}
			else 
			{
				USART_Cmd(USART6, DISABLE);
				p3_init(115200);
			}
		}
		
	}	
}



