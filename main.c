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

/* Driver configuration */
#include "ti/drivers/gpio/GPIOCC26XX.h"
#include "ti_drivers_config.h"
#include "epd.h"

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

    // // Sets up display
    // Display_Handle displayHandle;
    // Display_init();
    // displayHandle = Display_open(Display_Type_UART, NULL);
    // if (displayHandle == NULL)
    // {
    //     // Display_open() failed
    //     return (NULL);
    // } else {
    //     Display_printf(displayHandle, 0, 0, "Hi from ChromaAeon74! :D");
    // }

    // Sets up the NVS
    NVS_Handle nvsHandle;
    NVS_Params nvsParams;
    NVS_init();
    NVS_Params_init(&nvsParams);
    nvsHandle = NVS_open(CONFIG_NVS_FLASH, &nvsParams);
    if (nvsHandle == NULL)
    {
        // Display_printf(displayHandle, 0, 0, "NVS_open() failed.");
        return (NULL);
    }

    // Try reading a NVS and printing output
    for (int page=0; page < 1; page ++) {
        // Choose memory offset to read
        uint16_t byte_offset = page * sizeof(buf);

        // Read NVS
        NVS_read(nvsHandle, byte_offset, (void *)buf, sizeof(buf));

        // Print as ASCII
        char *ptr = &print_buf[0];
        for (int i = 0; i < sizeof(buf); i++) {
            ptr += sprintf(ptr, "%02x", buf[i]);
        }
        Display_printf(displayHandle, 0, 0, "%s", print_buf);
    }
    NVS_close(nvsHandle);


    //EPD
    // Display_printf(displayHandle, 0, 0, "INIT EPAPER...");
    Epd_Init();
    // Display_printf(displayHandle, 0, 0, "INIT EPAPER FINISHED!");
    Epd_DisplayPart(0,0,200,20);
    Epd_Sleep();

    // Display_printf(displayHandle, 0, 0, "Done!");


    return (NULL);
}
