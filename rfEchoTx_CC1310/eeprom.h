#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdbool.h>
#include <stdint.h>

bool eepromRead(uint32_t addr,void *dst,uint32_t len);
bool eepromWrite(uint32_t addr,void *src, uint32_t len);
bool eepromErase(uint32_t addr, uint32_t len);
bool eepromPowerDown(void);
bool eepromGetSFDP(void *pDst,uint32_t len);

uint32_t eepromGetSize(void);


#endif
