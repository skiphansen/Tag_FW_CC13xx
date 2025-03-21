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


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
   InitLogging();
   LOG("ChromaAeon74 compiled " __DATE__ " " __TIME__ "\n");

#if defined(NVS_TEST) || defined(NVS_DUMP)
    // Sets up the NVS
    NVS_Handle nvsHandle;
    NVS_Params nvsParams;
    NVS_Attrs regionAttrs;

    NVS_init();
    NVS_Params_init(&nvsParams);


    nvsHandle = NVS_open(CONFIG_NVS_FLASH, &nvsParams);
    if (nvsHandle == NULL)
    {
        // Display_printf(displayHandle, 0, 0, "NVS_open() failed.");
        return (NULL);
    }

#ifdef NVS_DUMP
    // Try reading a NVS and printing output
    for (int page=0; page < 0x100; page ++) {
        // Choose memory offset to read
        uint32_t byte_offset = page * sizeof(buf);

        // Read NVS
        NVS_read(nvsHandle, byte_offset, (void *)buf, sizeof(buf));

        // Print as ASCII
        char *ptr = &print_buf[0];
        for (int i = 0; i < sizeof(buf); i++) {
            ptr += sprintf(ptr, "%02x", buf[i]);
        }
        Display_printf(displayHandle, 0, 0, "%s", print_buf);
    }
#endif   // NVS_DUMP

#ifdef NVS_TEST
    do {
       int_fast16_t status;

       NVS_getAttrs(nvsHandle,&regionAttrs);

       LOG("regionSize 0x%x sectorSize 0x%x.",
           regionAttrs.regionSize,regionAttrs.sectorSize);
       // Fetch the generic NVS region attributes for nvsHandle
       NVS_getAttrs(nvsHandle, &regionAttrs);

#ifndef NVS_TEST_READBACK_ONLY
       // Erase the first sector of nvsHandle
       status = NVS_erase(nvsHandle, 0, regionAttrs.sectorSize);
       if (status != NVS_STATUS_SUCCESS) {
          LOG("NVS_erase failed %d\n",status);
          break;
       }

       // Write "Hello" to the base address of nvsHandle, verify after write
       status = NVS_write(nvsHandle, 0, "Hello", strlen("Hello")+1, NVS_WRITE_POST_VERIFY);
       if (status != NVS_STATUS_SUCCESS) {
          LOG("NVS_write failed %d\n",status);
          break;
       }
#endif

       // Copy "Hello" from nvsHandle into local 'buf'
       status = NVS_read(nvsHandle, 0, buf, strlen("Hello")+1);
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
       NVS_close(nvsHandle);

    } while(false);
#endif   // NVS_TEST

    NVS_close(nvsHandle);
#endif   // defined(NVS_TEST) || defined(NVS_DUMP)

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


    return (NULL);
}
