#ifndef _LOW_PASS_H
#define _LOW_PASS_H

#include "status.h"
#include "dead_zone.h"

typedef struct 
{
	float ch[4];
}LOW_PASS;

LOW_PASS Low_pass_output(DEAD_ZONE (*input)(P3_ch (*callback)(p3 *_ch)));



#endif


