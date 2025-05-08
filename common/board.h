#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef  CHROMA_AEON_74
#define BOARD_NAME                  "ChromaAeon74"
#define HW_TYPE                     0x81
#define FLASH_MX25V8006

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
#endif

#ifdef  CC1310_128K
#define BOARD_NAME                  "CC1310 128K"
#define FLASH_MX25V1006
#endif

#ifdef FLASH_MX25V8006
// Macronix 1 mbyte SPI flash parameters
#define EEPROM_SIZE                 0x100000
#define EEPROM_ERZ_SECTOR_SZ        0x1000   // erase size and alignment
#define EEPROM_WRITE_PAGE_SZ 256   // max write size & alignment
// #define CMD_ERASE_32K
// #define CMD_ERASE_64K
#endif

#ifdef FLASH_MX25V1006
// Macronix 128K SPI flash parameters
#define EEPROM_SIZE                 0x20000
#define EEPROM_ERZ_SECTOR_SZ        0x1000   // erase size and alignment
#define EEPROM_WRITE_PAGE_SZ 256   // max write size & alignment
// #define CMD_ERASE_32K
// #define CMD_ERASE_64K   0xd8
#endif

#endif   // _BOARD_H_
