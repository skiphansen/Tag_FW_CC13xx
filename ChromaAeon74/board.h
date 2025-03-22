#ifndef _BOARD_H_
#define _BOARD_H_

#define CHROMA29
#define BOARD_NAME "ChromaAeon74"
#define HW_TYPE    0x82

#define SPI_FLASH_SIZE                 0x100000
#define SPI_FLASH_ERZ_SECTOR_SZ        0x1000

#define SPI_FLASH_SETTINGS_AREA_START  0x0
#define SPI_FLASH_SETTINGS_AREA_LEN    SPI_FLASH_ERZ_SECTOR_SZ

#define SPI_FLASH_OTA_START            (0x94000)
#define SPI_FLASH_OTA_LEN              (352*1024)

#define SPI_FLASH_IMG_START            (0x4000)
#define SPI_FLASH_IMG_EACH             (0x24000)
#define SPI_FLASH_IMG_SECTORS          (SPI_FLASH_IMG_EACH / SPI_FLASH_ERZ_SECTOR_SZ)
#define IMAGE_SLOTS                    ((SPI_FLASH_SIZE - SPI_FLASH_IMG_START)/SPI_FLASH_IMG_EACH)

#define SN_LEN                         7
#define SN_OFFSET                      0x2000
#define SN_CHAR1                       'S'
#define SN_CHAR2                       'R'
#define SN_REV_CHAR                    'C'


#endif   // _BOARD_H_
