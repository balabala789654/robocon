#include "Low_pass.h"

static LOW_PASS Low_pass_output_last;
static float p = 0.002;


LOW_PASS Low_pass_output(DEAD_ZONE (*input)(P3_ch (*callback)(p3 *_ch)))
{
	LOW_PASS _low_pass;
	for(int i=0; i<4; i++)
		_low_pass.ch[i] = ((*input)(status_p3_ch_callback).ch[i]*p + (Low_pass_output_last.ch[i]*(1-p)));
	Low_pass_output_last = _low_pass;
	return _low_pass;
}




