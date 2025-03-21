/*
 * Copyright (c) 2017-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== nvsexternal.c ========
 */

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
#define FOOTER "=================================================="

/* Buffer placed in RAM to hold bytes read from non-volatile storage. */
#define BUF_SIZE 4096
static char buf[BUF_SIZE];
char print_buf[(BUF_SIZE * 2) + 1];

#define SN_LEN    7
#define SN_OFFSET    0x2000

NVS_Handle gNvs;
uint8_t mSelfMac[8];

void InitSN(void);


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
   InitLogging();
   LOG("ChromaAeon74 compiled " __DATE__ " " __TIME__ "\n");

   InitSN();

  // Sets up the NVS
    NVS_Params nvsParams;
    NVS_Attrs regionAttrs;

    NVS_init();
    NVS_Params_init(&nvsParams);


    gNvs = NVS_open(CONFIG_NVS_FLASH, &nvsParams);
    if (gNvs == NULL)
    {
        LOG("NVS_open() failed\n");
        return (NULL);
    }

#ifdef NVS_DUMP
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
        LOG("%s", print_buf);
    }
#endif   // NVS_DUMP

#ifdef NVS_TEST
    do {
       int_fast16_t status;

       NVS_getAttrs(gNvs,&regionAttrs);

       LOG("regionSize 0x%x sectorSize 0x%x.",
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

       // close the region
       NVS_close(gNvs);

    } while(false);
#endif   // NVS_TEST


#ifdef EPD_TEST
    // Init
    LOG("INIT EPAPER...");
    Epd_Init();
    LOG("INIT EPAPER FINISHED!");

    // Drawing
    LOG("Drawing...");
    Epd_Draw();
    LOG("Done!");

    // Sleep
    LOG("Sleeping epd!");
    Epd_Sleep();
    LOG("EOP");
#endif

    NVS_close(gNvs);

    return (NULL);
}

void InitSN()
{
   char SN[SN_LEN];

   memcpy(SN,(void *) SN_OFFSET,SN_LEN);

   if(SN[0] != 'S' || SN[1] != 'R' || SN[6] != 'C') {
      LOG("Invalid SN:\n");
      DUMP_HEX(SN,SN_LEN);
      LOG("Setting default SN\n");
      strcpy(SN,"SR0000000000C");
   }
   mSelfMac[7] = 0x44;
   mSelfMac[6] = 0x67;
   mSelfMac[3] = SN[2];
   mSelfMac[2] = SN[3];
   mSelfMac[1] = SN[4];
   mSelfMac[0] = SN[5];
// Since the first 2 characters are known to be upper case ASCII subtract
// 'A' from the value to convert it from a 7 bit value to a 5 bit value.
// 
// This allows us to use the extra 3 bits to convey something else like the
// hardware variation.
// 
// i.e. Multiple incompatible hardware variations that need different FW 
// images, but otherwise are compatible can share a single HWID and the 
// AP can parse the "extra" bits in the MAC address to select the 
// correct OTA image.

   mSelfMac[5] = SN[0] - 'A';
   mSelfMac[4] = SN[1] - 'A';
#ifdef HW_VARIANT
   mSelfMac[5] |= HW_VARIANT << 5;
   LOG("HW variant %d\n",HW_VARIANT);
#endif

   LOG("SN: %c%c%02x%02x%02x%02x%c\n",
       SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6]);

   LOG("MAC: ");
   DUMP_HEX(mSelfMac,sizeof(mSelfMac));
}

