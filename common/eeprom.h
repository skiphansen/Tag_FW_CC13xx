#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdbool.h>
#include <stdint.h>

bool eepromRead(uint32_t addr,void *dst,uint32_t len);
bool eepromWrite(uint32_t addr,void *src, uint32_t len);
bool eepromErase(uint32_t addr, uint32_t len);
bool eepromPowerDown(void);
bool eepromInit(void);

uint32_t eepromGetSize(void);

#define EEPROM_IMG_VALID				(0x494d4721UL)
#include "board.h"

struct EepromImageHeader {				//each image space is 0x17000 bytes, we have space for ten of them
	uint64_t version;
	uint32_t validMarker;
	uint32_t size;
	uint8_t dataType;
	uint32_t id;
	
	//image data here
	//we pre-erase so progress can be calculated by finding the first non-0xff byte
};



#endif
