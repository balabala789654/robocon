#ifndef _STATUS_H
#define _STATUS_H

#include "rabbit.h"

#define ON 1
#define OFF 0

enum
{
	Reset_system=1, 
	Status_pick_up,
	Status_chassis,
	Status_shoot_high,
	Status_shoot_mid,
	Status_shoot_low,
	Status_shoot_close
};
enum
{
	mode_0=0,
	mode_1,
	mode_2
};

void status_task(void *pvParameters);
void status_set(RABBIT *_rabbit, char state);
void status_mode_set(RC_ctrl_t *_rc, RABBIT *_rabbit);



#endif

