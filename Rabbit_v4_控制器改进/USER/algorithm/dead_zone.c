#include "dead_zone.h"


static char dead_zone_param=20;
 
 DEAD_ZONE dead_zone_output(P3_ch (*callback)(p3 *_ch))
{
	DEAD_ZONE _remote;
	_remote.ch[0]=dead_zone_change((*callback)(&p3_data).ch[0], dead_zone_param);
	_remote.ch[1]=dead_zone_change((*callback)(&p3_data).ch[1], dead_zone_param);
	_remote.ch[2]=dead_zone_change((*callback)(&p3_data).ch[2], dead_zone_param);
	_remote.ch[3]=dead_zone_change((*callback)(&p3_data).ch[3], dead_zone_param);

	return _remote;
}

int dead_zone_change(double _input, char _set)
{
	double _output;
	
	if(_input>=0)
	{
		_input-=_set;
		if(_input>=0)
			_output=_input;
		else _output=0;
	}
	else if(_input<0)
	{
		_input+=_set;
		if(_input<=0)
			_output=_input;
		else _output=0;
	}
	_output = _output*127/(127-_set);
	return _output;
	
}

 
