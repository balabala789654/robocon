#include "main.h"


#if FreeRtos
int main(void)
{
	
	rabbit_init();
	
	FreeRtos_start();
	
	while(1);
}

#else

int main(void)
{
	rabbit_init();
	
	set_chassis_max_speed(&rabbit.chassis, 1000, 1000);
	
	address_transform(&rabbit.chassis, (double* )&rc_ctrl.rc.ch[2], (double* )&rc_ctrl.rc.ch[3], (double* )&rc_ctrl.rc.ch[0]);

	while(1)
	{

		
		remote_contorl_speed(&rc_ctrl,&rabbit.chassis);
		motor_speed_compute(&rabbit.chassis);
		
		moter_send_3508(rabbit.chassis.chassis_motor[0].pid_param.out,
						rabbit.chassis.chassis_motor[1].pid_param.out,
						rabbit.chassis.chassis_motor[2].pid_param.out,
						rabbit.chassis.chassis_motor[3].pid_param.out);
		

	}
}

#endif

