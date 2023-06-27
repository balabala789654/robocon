#include "main.h"



/**
	*@brief µ÷ÊÔ½×¶ÎÊ¹ÓÃ
	*@param 
	*@param	
	*@retval 
**/
void function_task(void *pvParameters)
{
	
	while(1)
	{
		
		function(&rc_ctrl);
		vTaskDelay(1);
	}
	
}

void function(RC_ctrl_t* _rc)
{
	if(_rc->rc.s[0]==2&&_rc->rc.s[1]==2)
	{
		rabbit.rabbit_state = OFF;
		rabbit.chassis.chassis_switch = OFF;
		rabbit.clamping.clamping_switch = OFF;
		rabbit.shoot.shoot_switch = OFF;
	}
	else 
	{
		rabbit.rabbit_state = ON;
		if(_rc->rc.s[1]==3) rabbit.chassis.chassis_switch = ON;
		else rabbit.chassis.chassis_switch = OFF;
		
		if(_rc->rc.s[1]==1)
		{
			if(_rc->rc.s[0]==2)
			{
				rabbit.clamping.clamping_switch = ON;
				rabbit.shoot.shoot_switch = OFF;
			}
			else if(_rc->rc.s[0]==3)
			{	rabbit.clamping.clamping_switch = OFF;
				rabbit.shoot.shoot_switch = ON;
			}
			else 
			{
				rabbit.clamping.clamping_switch = OFF;
				rabbit.shoot.shoot_switch = OFF;
			}
		}
		else 
		{
			rabbit.clamping.clamping_switch = OFF;
			rabbit.shoot.shoot_switch = OFF;
		}

	}
//	else 
//	{
//		rabbit.rabbit_state = rabbit_unlocked;
//		if(_rc->rc.s[1]==2)
//		{
//			rabbit.clamping.clamping_switch = rabbit_clamping_on;
//		}
//	}
	
	


}


