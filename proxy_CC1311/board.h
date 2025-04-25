#ifndef _BOARD_H_
#define _BOARD_H_
#include "cc13xx.h"

#define CHROMA29
#define BOARD_NAME                  "ChromaAeon74"
#define HW_TYPE                     0x81

#define EEPROM_SIZE                 0x100000
#define EEPROM_ERZ_SECTOR_SZ        0x1000

#define EEPROM_SETTINGS_AREA_START  0x0
#define EEPROM_SETTINGS_AREA_LEN    EEPROM_ERZ_SECTOR_SZ

#define EEPROM_OTA_START            (0x94000)
#define EEPROM_OTA_LEN              (352*1024)

#define EEPROM_IMG_START            (0x10000)
#define EEPROM_IMG_EACH             (0x20000)
#define EEPROM_IMG_SECTORS          (EEPROM_IMG_EACH / EEPROM_ERZ_SECTOR_SZ)
#define IMAGE_SLOTS                 ((EEPROM_SIZE - EEPROM_IMG_START)/EEPROM_IMG_EACH)

#define SN_LEN                      7
#define SN_OFFSET                   0x2000
#define SN_CHAR1                    'S'
#define SN_CHAR2                    'R'
#define SN_REV_CHAR                 'C'

#endif   // _BOARD_H_
