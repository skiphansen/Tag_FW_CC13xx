/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 *  ======== CC1310_LAUNCHXL_NoRTOS.cmd ========
 */

--retain=g_pfnVectors

/* C stack is also used for ISR stack. 1.5 KB: inflating a zlib image's
   Huffman trees (uzlib) needs ~0.4 KB on top of the download path */
--stack_size=1536

--heap_size=256

/* Override default entry point.                                             */
--entry_point resetISR
/* Allow main() to take args                                                 */
--args 0x8
/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application.                      */
/* The Chroma tags use a CC1310F64. For a CC1310F128 tag pass                */
/* --define=CC1310F128 to the linker. (Not CC1310_128K: that compiler define */
/* selects a 128K *SPI flash* in common/oepl_board.h.)                            */
/* --define=OEPL_APP: the Chroma 74H OEPL app, started by bootloader_CC1310.  */
/* Sector 0 holds the bootloader and sector 15 the CCFG (part of the         */
/* bootloader image); OTA only ever writes 0x1000-0xEFFF.                    */
#define RAM_BASE                0x20000000
#ifdef OEPL_APP
#define FLASH_BASE              0x1000
#define FLASH_SIZE              0xE000
#define RAM_SIZE                0x4000
#else
#define FLASH_BASE              0x0
#ifdef CC1310F128
#define FLASH_SIZE              0x20000
#define RAM_SIZE                0x5000
#else
#define FLASH_SIZE              0x10000
#define RAM_SIZE                0x4000
#endif
#endif

/* System memory map */

MEMORY
{
    /* Application stored in and executes from internal flash */
    FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE
    /* Application uses internal RAM for data */
    SRAM (RWX) : origin = RAM_BASE, length = RAM_SIZE
}

/* Section allocation in memory */

SECTIONS
{
    .intvecs        :   > FLASH_BASE
    .text           :   > FLASH
    .TI.ramfunc     : {} load=FLASH, run=SRAM, table(BINIT)
    .const          :   > FLASH
    .constdata      :   > FLASH
    .rodata         :   > FLASH
    .cinit          :   > FLASH
    .pinit          :   > FLASH
    .init_array     :   > FLASH
    .emb_text       :   > FLASH
#ifndef OEPL_APP
    .ccfg           :   > FLASH (HIGH)
#endif

    .vtable_ram     :   > SRAM
    .data           :   > SRAM
    /* >> lets .bss split around the fixed .TI.noinit below */
    .bss            :   >> SRAM
    .sysmem         :   > SRAM
    .nonretenvar    :   > SRAM
    /* #pragma NOINIT variables, not touched by the C startup (the watchdog
       reset marker in oepl_74h.c). Behind bootloader_CC1310 they must also
       stay clear of the RAM its startup zeroes on every boot (OTA_BOOT_RAM_*
       in ota_74h.h), or no record survives a reset. */
#ifdef OEPL_APP
    .TI.noinit      :   > 0x20001800
#else
    .TI.noinit      :   > SRAM
#endif

    .stack          :   > SRAM (HIGH)
}
