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
void *controllerThread(void *arg0)
{
    SPI_Handle controllerSpi;
    SPI_Params spiParams;
    SPI_Transaction transaction;
    uint32_t i;
    bool transferOK;
    int32_t status;

    /* Open SPI as controller (default) */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    /* See device-specific technical reference manual for supported speeds */
    spiParams.bitRate     = 1000000;
    controllerSpi         = SPI_open(CONFIG_SPI_CONTROLLER, &spiParams);
    if (controllerSpi == NULL)
    {
        LOG("Error initializing controller SPI\n");
        while (1) {}
    }
    else
    {
        LOG("Controller SPI initialized\n");
    }


        /* Initialize controller SPI transaction structure */
        controllerTxBuffer[sizeof(CONTROLLER_MSG) - 1] = (i % 10) + '0';
        memset((void *)controllerRxBuffer, 0, SPI_MSG_LENGTH);
        transaction.count = SPI_MSG_LENGTH;
        transaction.txBuf = (void *)controllerTxBuffer;
        transaction.rxBuf = (void *)controllerRxBuffer;

        /* Toggle user LED, indicating a SPI transfer is in progress */
        GPIO_toggle(CONFIG_GPIO_LED_1);

        /* Perform SPI transfer */
        transferOK = SPI_transfer(controllerSpi, &transaction);
        if (transferOK)
        {
            Display_printf(display, 0, 0, "Controller received: %s", controllerRxBuffer);
        }
        else
        {
            Display_printf(display, 0, 0, "Unsuccessful controller SPI transfer");
        }

        /* Sleep for a bit before starting the next SPI transfer  */
        sleep(3);
    }

    SPI_close(controllerSpi);

    /* Example complete - set pins to a known state */
    GPIO_disableInt(CONFIG_SPI_PERIPHERAL_READY);
    GPIO_setConfig(CONFIG_SPI_PERIPHERAL_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_SPI_CONTROLLER_READY, 0);

    Display_printf(display, 0, 0, "\nDone");

    return (NULL);
}
#endif   // SPI_TEST

#ifdef __cplusplus
#pragma message ( "undefing __cplusplus" )
#undef __cplusplus
#endif

#ifdef __cplusplus
#pragma message ( "__cplusplus still defined!" )
#endif

#define  _LINUX_  // make bbe_paper code happy
#include "bb_epaper.h"
#include "oepl_io.inl"
#include "bb_ep.inl"
#include "bb_ep_gfx.inl"

BBEPDISP bbep; // the main display structure

void bbTest()
{
   const char Msg[] = "bb_epaper on OEPL!";
   int x = 400 - (((sizeof(Msg) - 1) * 16) / 2);
   ELOG("x = %d\n",x);

   GPIO_write(PWR_PIN,0);
   bbepSetPanelType(&bbep,EP75R_800x480); // must set this first
   bbepInitIO(&bbep,8000000);
//   bbepSetRotation(&bbep,90);
   bbepFill(&bbep,BBEP_WHITE, 0);
   bbepFill(&bbep,BBEP_WHITE, 1);
   bbepWriteString(&bbep,x,240,Msg,FONT_16x16,BBEP_BLACK);
   ELOG("Start refresh\n");
   bbepRefresh(&bbep,REFRESH_FULL);
   bbepWaitBusy(&bbep);
   bbepSleep(&bbep,DEEP_SLEEP);
   LOG("Done\n");
}

