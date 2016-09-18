#ifndef _TIEMR0_
#define _TIEMR0_
	#include "typeDef.h"
	#include "STC15.h"
	#define timer0_clear()	TH0 = TL0 = 0X00
	#define timer0_start()	TR0 = 1
	#define timer0_stop()	TR0 = 0
	extern u16 timer0_read(void);
	extern void timer0_init(void);
#endif