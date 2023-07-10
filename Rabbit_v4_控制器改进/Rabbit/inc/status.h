#ifndef _STATUS_H
#define _STATUS_H

#include "p3.h"

#define ON 1
#define OFF 0

#define UP 1
#define DOWN 0
#define MID 2
typedef struct
{
	int ch[4];
}P3_ch;

typedef struct
{
	char L2;
	char L1;
	char LU;
	char LL;
	char LD;
	char LR;
	char SE;
	char ST;
	char RL;
	char RD;
	char RR;
	char RU;
	char R1;
	char R2;
	char R_key;
	char L_key;	
}P3_key;

P3_ch status_p3_ch_callback(p3 *_ch);
P3_key status_p3_key_callback(p3 *_key);

#endif

