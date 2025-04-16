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
#include "logging.h"

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
}
#endif


#ifdef WRITE_EPD_IMAGE
extern const uint8_t img_weather[];

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

      for(int i = 0; i < 0x17700; i++) {
         gTempBuf[Wr++] = img_weather[i];
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

#ifdef SPI_TEST
#include <ti/drivers/SPI.h>

void SpiTest()
{
   SPI_Handle controllerSpi;
   SPI_Params spiParams;
   SPI_Transaction transaction;
   uint32_t i;
   bool transferOK;
   int32_t status;
   int_fast16_t Err;
   uint8_t Wake_Cmd = 0xab;
   uint8_t SPDP_CmdBuf[] = {
      0x5a,    // Cmd
      0, 0, 0, // daddr
      0,       // dummy
      0,0,0,0,0,0,0,0   // send zeros while reading data
   };
   uint8_t SPDP_RxBuf[sizeof(SPDP_CmdBuf)];

   LOG("Called\n");
   SPI_init();

   memset(SPDP_RxBuf,0xaa,sizeof(SPDP_RxBuf));
   SPI_Params_init(&spiParams);
//   spiParams.bitRate      = 4000000;
   spiParams.bitRate      = 100000;
   spiParams.mode         = SPI_CONTROLLER;
   spiParams.transferMode = SPI_MODE_BLOCKING;
   // spiParams.frameFormat = SPI_POL0_PHA1;
   do {
      if((controllerSpi = SPI_open(CONFIG_SPI_FLASH,&spiParams)) == NULL) {
         LOG("Error initializing controller SPI\n");
         break;
      }
      LOG("Controller SPI initialized\n");
      Err = GPIO_setConfig(CONFIG_GPIO_NVS_FLASH_CS,
                           GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
      if(Err != GPIO_STATUS_SUCCESS) {
         LOG("GPIO_setConfig failed %d\n",Err);
         break;
      }

   // Wake EEPROM from sleep
      transaction.count = 1;
      transaction.txBuf = (void *)&Wake_Cmd;
      transaction.rxBuf = (void *)SPDP_RxBuf;

       /* Perform SPI transfer */
      GPIO_write(CONFIG_GPIO_NVS_FLASH_CS_CONST,0);
      transferOK = SPI_transfer(controllerSpi, &transaction);
      GPIO_write(CONFIG_GPIO_NVS_FLASH_CS_CONST,1);
      if(!transferOK) {
         LOG("SPI_transfer failed:\n");
         break;
      }
      ClockP_usleep(100);


      transaction.count = sizeof(SPDP_CmdBuf);
      transaction.txBuf = (void *)SPDP_CmdBuf;
      transaction.rxBuf = (void *)SPDP_RxBuf;

      LOG("Cmd:\n");
      DUMP_HEX(SPDP_CmdBuf,sizeof(SPDP_CmdBuf));

       /* Perform SPI transfer */
      GPIO_write(CONFIG_GPIO_NVS_FLASH_CS_CONST,0);
      transferOK = SPI_transfer(controllerSpi, &transaction);
      GPIO_write(CONFIG_GPIO_NVS_FLASH_CS_CONST,1);
      if(!transferOK) {
         LOG("SPI_transfer 1 failed\n");
         break;
      }
      LOG("Read:\n");
      DUMP_HEX(SPDP_RxBuf,sizeof(SPDP_RxBuf));
   } while(false);

   SPI_close(controllerSpi);
   LOG("Done\n");
   while(true);
}
#endif   // SPI_TEST
