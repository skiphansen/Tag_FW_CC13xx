#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/display/Display.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti/drivers/gpio/GPIOCC26XX.h"
#include "ti_drivers_config.h"
#include "epd.h"
#include "img.h"
#include "main.h"
#include "img.h"
#include "board.h"

#ifdef NVS_DUMP

/* Buffer placed in RAM to hold bytes read from non-volatile storage. */
#define BUF_SIZE 4096
char buf[BUF_SIZE];
char print_buf[(TEMP_BUF_SIZE * 2) + 1];

void NvrDump()
{
    // Try reading a NVS and printing output
    for (int page=0; page < 0x100; page ++) {
        // Choose memory offset to read
        uint32_t byte_offset = page * sizeof(buf);

        // Read NVS
        NVS_read(gNvs, byte_offset, (void *)buf, sizeof(buf));

        // Print as ASCII
        char *ptr = &print_buf[0];
        for (int i = 0; i < sizeof(buf); i++) {
            ptr += sprintf(ptr, "%02x", buf[i]);
        }
        LOG("%s\n", print_buf);
    }
}
#endif   // NVS_DUMP

#ifdef NVS_TEST
void NvrTest()
{
    do {
       int_fast16_t status;

       NVS_getAttrs(gNvs,&regionAttrs);

       LOG("regionSize 0x%x sectorSize 0x%x.\n",
           regionAttrs.regionSize,regionAttrs.sectorSize);
       // Fetch the generic NVS region attributes for gNvs
       NVS_getAttrs(gNvs, &regionAttrs);

#ifndef NVS_TEST_READBACK_ONLY
       // Erase the first sector of gNvs
       status = NVS_erase(gNvs, 0, regionAttrs.sectorSize);
       if (status != NVS_STATUS_SUCCESS) {
          LOG("NVS_erase failed %d\n",status);
          break;
       }

       // Write "Hello" to the base address of gNvs, verify after write
       status = NVS_write(gNvs, 0, "Hello", strlen("Hello")+1, NVS_WRITE_POST_VERIFY);
       if (status != NVS_STATUS_SUCCESS) {
          LOG("NVS_write failed %d\n",status);
          break;
       }
#endif

       // Copy "Hello" from gNvs into local 'buf'
       status = NVS_read(gNvs, 0, buf, strlen("Hello")+1);
       if (status != NVS_STATUS_SUCCESS) {
           // Error handling code
          LOG("NVS_read failed %d\n",status);
          break;
       }

       // Print the string from fetched NVS storage
       LOG("readback '%s'\n", buf);
       if (regionAttrs.regionBase == NVS_REGION_NOT_ADDRESSABLE) {  
          LOG("NVS_REGION_NOT_ADDRESSABLE\n");
       }
    } while(false);
}
#endif   // NVS_TEST

#ifdef EPD_TEST
void EpdTest()
{
    // Init
    LOG("INIT EPAPER...\n");
    Epd_Init();
    LOG("INIT EPAPER FINISHED!\n");

    // Drawing
    LOG("Drawing...");
    Epd_Draw();
    LOG("\nDone!");

    // Sleep
    LOG("Sleeping epd!\n");
    Epd_Sleep();
    LOG("EOP\n");
#endif


#ifdef WRITE_EPD_IMAGE
void WriteEpdImage()
{
   int_fast16_t status;
   uint32_t WrOffset = SPI_FLASH_IMG_START;
   uint32_t Wr = 0;
   NVS_Attrs regionAttrs;

   do {
      NVS_getAttrs(gNvs,&regionAttrs);

      LOG("regionSize 0x%x sectorSize 0x%x.\n",
          regionAttrs.regionSize,regionAttrs.sectorSize);
      // Fetch the generic NVS region attributes for gNvs
      NVS_getAttrs(gNvs, &regionAttrs);

      size_t EraseLen = SPI_FLASH_IMG_EACH;
      if((SPI_FLASH_IMG_EACH % SPI_FLASH_ERZ_SECTOR_SZ) != 0) {
         LOG("Error: SPI_FLASH_IMG_EACH not multiple of SPI_FLASH_ERZ_SECTOR_SZ\n");
         break;
      }
      LOG("Erasing 0x%x bytes starting at 0x%x\n",EraseLen,WrOffset);

      status = NVS_erase(gNvs,WrOffset,EraseLen);
      if (status != NVS_STATUS_SUCCESS) {
         LOG("NVS_erase failed %d\n",status);
         break;
      }

      uint32_t Width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8) : (EPD_WIDTH / 8 + 1);  // Width in bytes, rounding up to handle partial bytes
      uint32_t Height = EPD_HEIGHT;
      uint32_t imgWidthInBytes = (IMG_WIDTH % 8 == 0) ? (IMG_WIDTH / 8) : (IMG_WIDTH / 8 + 1);  // Width of the image in bytes

      // Calculate the horizontal and vertical offsets to center the image
      uint32_t horizontal_offset = (EPD_WIDTH - IMG_WIDTH) / 2;
      uint32_t vertical_offset = (EPD_HEIGHT - IMG_HEIGHT) / 2;

      uint32_t i, j;
      uint8_t output;

      LOG("\nSending secondary color @ 0x%x .",WrOffset);

      for (j = 0; j < Height; j++) {
          for (i = 0; i < Width; i++) {
              if (i >= horizontal_offset / 8 && i < (horizontal_offset + IMG_WIDTH) / 8 && j >= vertical_offset && j < (vertical_offset + IMG_HEIGHT)) {
                  // If within the bounds of the image data, draw from img_data
                  uint32_t imgIndex = (j - vertical_offset) * imgWidthInBytes + (i - horizontal_offset / 8);
                  output = (imgIndex < IMG_DATA_SIZE) ? img_data[imgIndex] : 0xFF;  // Ensure we don't go out of bounds
              } else {
                  // Else, fill with white (0xFF), which corresponds to 0b1 bits for each byte
                  output = 0xFF;
              }

              gTempBuf[Wr++] = output;
              if(Wr == SPI_FLASH_ERZ_SECTOR_SZ) {
              // Flush sector worth of data to flash
                 status = NVS_write(gNvs,WrOffset,gTempBuf,SPI_FLASH_ERZ_SECTOR_SZ,NVS_WRITE_POST_VERIFY);
                 if (status != NVS_STATUS_SUCCESS) {
                    LOG("NVS_write failed at 0x%x, %d\n",WrOffset,status);
                    break;
                 }
                 LOG(".");
                 WrOffset += SPI_FLASH_ERZ_SECTOR_SZ;
                 Wr = 0;
              }
          }
      }

      LOG("\nSending secondary color @ 0x%x .",WrOffset);
      for(i=0; i<Width*Height; i++) {
         gTempBuf[Wr++] = output;
         if(Wr == SPI_FLASH_ERZ_SECTOR_SZ) {
         // Flush sector worth of data to flash
            status = NVS_write(gNvs,WrOffset,gTempBuf,SPI_FLASH_ERZ_SECTOR_SZ,NVS_WRITE_POST_VERIFY);
            if (status != NVS_STATUS_SUCCESS) {
               LOG("NVS_write failed at 0x%x, %d\n",WrOffset,status);
               break;
            }
            LOG(".");
            WrOffset += SPI_FLASH_ERZ_SECTOR_SZ;
            Wr = 0;
         }
      }

      if(Wr != 0) {
      // Flush sector worth of data to flash
         LOG("\nFlushing last %d bytes\n",Wr);

         status = NVS_write(gNvs,WrOffset,gTempBuf,SPI_FLASH_ERZ_SECTOR_SZ,NVS_WRITE_POST_VERIFY);
         if (status != NVS_STATUS_SUCCESS) {
            LOG("NVS_write failed at 0x%x, %d\n",WrOffset,status);
            break;
         }
      }
      else {
         LOG("\n");
      }
   } while(false);
}
#endif   // WRITE_EPD_IMAGE

