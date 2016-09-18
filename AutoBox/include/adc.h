#ifndef _ADC_
#define _ADC_
	#include "typeDef.h"
	#include "STC15.h"
	#include "intrins.h"
	
	extern void adc_init(void);
	extern void Delay(u16 n);
	extern u8 GetADCResult(void);
#endif