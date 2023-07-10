#ifndef _DEAD_ZONE_H
#define _DEAD_ZONE_H

#include "status.h"

typedef struct
{
    int ch[4];
}DEAD_ZONE;

int dead_zone_change(double _input, char _set);

DEAD_ZONE dead_zone_output(P3_ch (*callback)(p3 *_ch));


#endif



