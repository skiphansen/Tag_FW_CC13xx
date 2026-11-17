/*
 * Chroma 74H OTA: the contract between the OEPL app and bootloader_CC1310.
 *
 * Internal flash (CC1310F64, 4 KB sectors):
 *    0x0000-0x0fff  bootloader (never written by OTA)
 *    0x1000-0xefff  app, linked at OTA_APP_BASE (--define=OEPL_APP)
 *    0xf000-0xffff  CCFG at 0xffa8 (part of the bootloader image, never
 *                   written by OTA)
 *
 * The app downloads an OTA file (OtaHeader + image) into SPI flash at
 * OTA_STAGE_ADR, checks it, writes an OtaRecord at OTA_RECORD_ADR and
 * resets. On every boot the bootloader looks at the record; an armed record
 * that isn't done gets the image CRC checked, the image copied to 0x1000
 * and the CRC checked again before the record is marked done. A reset
 * during the copy repeats it on the next boot.
 *
 * Keep this layout stable: a new app has to work with the bootloader that
 * is already on the tag.
 */
#ifndef _OTA_74H_H_
#define _OTA_74H_H_

#include <stdint.h>

#define OTA_BOOT_VERSION     1

#define OTA_APP_BASE         0x1000
#define OTA_APP_MAX          0xe000      // up to the CCFG sector
#define OTA_SECTOR_SIZE      0x1000

// RAM the bootloader's startup zeroes or uses on every boot (its .bss and
// stack). The app's #pragma NOINIT records (watchdog marker, fault record,
// EPD trace) are linked at 0x20001800, outside both (Chroma_CC1310.cmd).
#define OTA_BOOT_RAM_LOW_END  0x20001000  // 0x20000000 up to here
#define OTA_BOOT_RAM_HIGH     0x20003e00  // from here to 0x20004000

// SPI flash (EEPROM_OTA_START is 0x10000, 64 KB)
#define OTA_STAGE_ADR        0x10000     // the OTA file as downloaded
#define OTA_RECORD_ADR       0x1f000     // last sector of the OTA area

#define OTA_FILE_MAGIC       0x3437414fUL   // "OA74"
typedef struct {
   uint32_t Magic;
   uint8_t HwType;
   uint8_t HeaderLen;      // sizeof(OtaHeader)
   uint16_t Version;       // FW_VERSION of the image
   uint32_t ImageLen;      // bytes from OTA_APP_BASE on
   uint32_t ImageCrc;      // CRC32 (zlib) of the image
   uint32_t HeaderCrc;     // CRC32 of the fields above
} OtaHeader;

// Written by the app once the staged file checks out. Armed is programmed
// last, Done by the bootloader after a verified copy (both 0xff -> 0x00,
// no erase needed).
#define OTA_RECORD_MAGIC     0x594c5041UL   // "APLY"
typedef struct {
   uint32_t Magic;
   uint32_t ImageAdr;      // SPI flash address of the image
   uint32_t ImageLen;
   uint32_t ImageCrc;
   uint16_t Version;
   uint8_t Armed;          // 0x00 = apply
   uint8_t Done;           // 0x00 = applied
} OtaRecord;

#define OTA_ARMED_OFFSET     18
#define OTA_DONE_OFFSET      19
typedef char OtaRecordSize[(sizeof(OtaRecord) == 20) ? 1 : -1];
typedef char OtaHeaderSize[(sizeof(OtaHeader) == 20) ? 1 : -1];

// Bitwise CRC32 (zlib polynomial), no table: small for the bootloader
static inline uint32_t OtaCrc32(uint32_t Crc,const uint8_t *p,uint32_t Len)
{
   int i;

   Crc = ~Crc;
   while(Len--) {
      Crc ^= *p++;
      for(i = 0; i < 8; i++) {
         Crc = (Crc >> 1) ^ (0xedb88320UL & (0 - (Crc & 1)));
      }
   }
   return ~Crc;
}

#endif   // _OTA_74H_H_
