#include "status.h"

char status_change_callback(RC_ctrl_t *_rc);

static char status_last_1;
static char status_last_2;

void status_task(void *pvParameters)
{
	char (*arr)(RC_ctrl_t *_rc);
	while(1)
	{
		status_mode_set(&rc_ctrl, &rabbit);
		if(status_last_1 != rc_ctrl.rc.s[0] || status_last_2 != rc_ctrl.rc.s[1])
		{
			arr = status_change_callback;
			status_set(&rabbit, (*arr)(&rc_ctrl));
		}
		vTaskDelay(1);
	}
}

char status_change_callback(RC_ctrl_t *_rc)
{
	if(_rc->rc.s[0]==2&&_rc->rc.s[1]==2)
		return Reset_system;
	else if(_rc->rc.s[0]==2&&_rc->rc.s[1]==1)
		return 0;
	else if(_rc->rc.s[0]==2&&_rc->rc.s[1]==3)
		return 0;
	else if(_rc->rc.s[0]==1&&_rc->rc.s[1]==1)
		return Status_shoot_low;
	else if(_rc->rc.s[0]==1&&_rc->rc.s[1]==3)
		return Status_shoot_mid;
	else if(_rc->rc.s[0]==1&&_rc->rc.s[1]==2)
		return Status_shoot_high;
	else if(_rc->rc.s[0]==3&&_rc->rc.s[1]==1)
		return Status_chassis;
	else if(_rc->rc.s[0]==3&&_rc->rc.s[1]==3)
		return Status_chassis;
	else if(_rc->rc.s[0]==3&&_rc->rc.s[1]==2)
		return Status_chassis;
	
	else
		return 0;
}
void status_mode_set(RC_ctrl_t *_rc, RABBIT *_rabbit)
{
	if(_rc->rc.ch[4]>=500)
		_rabbit->mode = mode_1;
	else if(_rc->rc.ch[4]<=-500)
		_rabbit->mode = mode_2;
	else 
		_rabbit->mode = mode_0;
}

void status_set(RABBIT *_rabbit, char state)
{
	_rabbit->status = state;
}





