#ifndef _EEPROM_
#define _EEPROM_
	typedef unsigned char BYTE;
	typedef unsigned int WORD;
	extern BYTE IapReadByte(WORD addr);
	extern void IapProgramByte(WORD addr, BYTE dat);
	extern void IapEraseSector(WORD addr);
#endif