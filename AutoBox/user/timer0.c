#include "timer0.h"

u16 timer0_read(void)
{
	u16 temp;
	temp = TH0;
	temp <<= 8;
	temp |= TL0;
	return temp;
}
void timer0_init(void)
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xf0;
	TMOD |= 0x01;
	ET0 = 0;
	TF0 = 0;
	timer0_clear();	
}