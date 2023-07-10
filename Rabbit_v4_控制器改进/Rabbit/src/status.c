#include "status.h"

P3_ch status_p3_ch_callback(p3 *_ch)
{
	P3_ch _p3;
	_p3.ch[0]=_ch->ch[0];
	_p3.ch[1]=_ch->ch[1];
	_p3.ch[2]=_ch->ch[2];
	_p3.ch[3]=_ch->ch[3];
	return _p3;
}

P3_key  status_p3_key_callback(p3 *_key)
{
	P3_key _p3_key;
	_p3_key.L1=_key->L1;
	_p3_key.L2=_key->L2;
	_p3_key.R1=_key->R1;
	_p3_key.R2=_key->R2;
	                 
	_p3_key.LD=_key->LD;
	_p3_key.LU=_key->LU;
	_p3_key.LL=_key->LL;
	_p3_key.LR=_key->LR;
	_p3_key.RD=_key->RD;
	_p3_key.RU=_key->RU;
	_p3_key.RL=_key->RL;
	_p3_key.RR=_key->RR;
	                 
	_p3_key.L_key=_key->L_key;
	_p3_key.R_key=_key->R_key;
	_p3_key.SE=_key->SE;
	_p3_key.ST=_key->ST;
	
	return _p3_key;
}

