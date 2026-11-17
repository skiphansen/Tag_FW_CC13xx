/*
 * Chroma 74H OTA, app side: check the firmware file the AP sent (staged in
 * SPI flash by syncedproto's downloadFWUpdate()), arm the bootloader and
 * reset into it. See ota_74h.h.
 */
#include "oepl_board.h"

#if defined(OEPL_74H) && defined(OEPL_OTA)
#include <stddef.h>
#include <string.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include "main.h"
#include "eeprom.h"
#include "syncedproto.h"
#include "ota_74h.h"
#include "logging.h"

// The copy must not brown out halfway (it would be repeated, but a weak
// battery would keep failing it)
#define OTA_MIN_MV         2400

void write_ota_firmware_to_flash(void)
{
   OtaHeader Hdr;
   OtaRecord Rec;
   uint32_t Crc = 0;
   uint32_t Done;
   uint32_t Len;
   uint32_t Vectors[2];
   uint8_t Zero = 0;

   wdt60s();
   if(eepromRead(OTA_STAGE_ADR,&Hdr,sizeof(Hdr))) {
      ELOG("OTA: header read failed\n");
      return;
   }
   if(Hdr.Magic != OTA_FILE_MAGIC || Hdr.HeaderLen != sizeof(Hdr)
      || Hdr.HwType != HW_TYPE
      || OtaCrc32(0,(const uint8_t *) &Hdr,offsetof(OtaHeader,HeaderCrc)) != Hdr.HeaderCrc
      || Hdr.ImageLen < sizeof(Vectors) || Hdr.ImageLen > OTA_APP_MAX)
   {
      ELOG("OTA: not a Chroma 74H image (magic 0x%x hwType 0x%x len %u)\n",
           Hdr.Magic,Hdr.HwType,Hdr.ImageLen);
      return;
   }
   if(Hdr.Version == FW_VERSION) {
      ALOG("OTA: version %d already running\n",Hdr.Version);
      return;
   }
// The new image's vector table: SP in RAM, reset handler inside the app
   eepromRead(OTA_STAGE_ADR + sizeof(Hdr),Vectors,sizeof(Vectors));
   if(Vectors[0] <= 0x20000000 || Vectors[0] > 0x20004000
      || !(Vectors[1] & 1) || Vectors[1] <= OTA_APP_BASE
      || Vectors[1] >= OTA_APP_BASE + Hdr.ImageLen)
   {
      ELOG("OTA: bad vectors 0x%x 0x%x\n",Vectors[0],Vectors[1]);
      return;
   }
   for(Done = 0; Done < Hdr.ImageLen; Done += Len) {
      Len = Hdr.ImageLen - Done;
      if(Len > TEMP_BUF_SIZE) {
         Len = TEMP_BUF_SIZE;
      }
      if(eepromRead(OTA_STAGE_ADR + sizeof(Hdr) + Done,gTempBuf,Len)) {
         ELOG("OTA: read failed\n");
         return;
      }
      Crc = OtaCrc32(Crc,gTempBuf,Len);
   }
   if(Crc != Hdr.ImageCrc) {
      ELOG("OTA: image CRC 0x%x, expected 0x%x\n",Crc,Hdr.ImageCrc);
      return;
   }
// The image already running, whatever version the file claims: applying it
// again would repeat forever if the AP keeps offering it
   if(OtaCrc32(0,(const uint8_t *) OTA_APP_BASE,Hdr.ImageLen) == Hdr.ImageCrc) {
      ALOG("OTA: this image is already running\n");
      return;
   }
// Not batteryVoltage: with REPORT_REFRESH_MV that can be the last refresh's
// minimum, far below what the cells give for the internal flash copy
   if(batteryRestMv < OTA_MIN_MV) {
      ELOG("OTA: battery %u mV too low\n",batteryRestMv);
      return;
   }

   memset(&Rec,0xff,sizeof(Rec));
   Rec.Magic = OTA_RECORD_MAGIC;
   Rec.ImageAdr = OTA_STAGE_ADR + sizeof(Hdr);
   Rec.ImageLen = Hdr.ImageLen;
   Rec.ImageCrc = Hdr.ImageCrc;
   Rec.Version = Hdr.Version;
   if(eepromErase(OTA_RECORD_ADR,OTA_SECTOR_SIZE)
      || eepromWrite(OTA_RECORD_ADR,&Rec,sizeof(Rec))
   // armed last: a reset before this leaves the record ignored
      || eepromWrite(OTA_RECORD_ADR + OTA_ARMED_OFFSET,&Zero,1))
   {
      ELOG("OTA: record write failed\n");
      return;
   }
   eepromPower(false);        // waits for the last write to finish
   ALOG("OTA: version %d (%u bytes) armed, resetting into the bootloader\n",
        Hdr.Version,Hdr.ImageLen);
   SysCtrlSystemReset();
}
#endif   // OEPL_74H && OEPL_OTA
