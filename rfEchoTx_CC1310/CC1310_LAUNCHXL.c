/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 *  ====================== CC1310_LAUNCHXL.c ===================================
 *  This file is responsible for setting up the board specific items for the
 *  CC1310_LAUNCHXL board.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/udma.h>
#include <ti/devices/cc13x0/inc/hw_ints.h>
#include <ti/devices/cc13x0/inc/hw_memmap.h>

#include "Board.h"

/*
 *  =============================== ADCBuf ===============================
 */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>

ADCBufCC26XX_Object adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBUFCOUNT];

/*
 *  This table converts a virtual adc channel into a dio and internal analogue
 *  input signal. This table is necessary for the functioning of the adcBuf
 *  driver. Comment out unused entries to save flash. Dio and internal signal
 *  pairs are hardwired. Do not remap them in the table. You may reorder entire
 *  entries. The mapping of dio and internal signals is package dependent.
 */
const ADCBufCC26XX_AdcChannelLutEntry ADCBufCC26XX_adcChannelLut[CC1310_LAUNCHXL_ADCBUF0CHANNELCOUNT] = {
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
};

const ADCBufCC26XX_HWAttrs adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBUFCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .adcChannelLut     = ADCBufCC26XX_adcChannelLut,
    }
};

const ADCBuf_Config ADCBuf_config[CC1310_LAUNCHXL_ADCBUFCOUNT] = {
    {
        &ADCBufCC26XX_fxnTable,
        &adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBUF0],
        &adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = CC1310_LAUNCHXL_ADCBUFCOUNT;

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

ADCCC26XX_Object adcCC26xxObjects[CC1310_LAUNCHXL_ADCCOUNT];


const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCCOUNT] = {
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_DCOUPL,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VSS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VDDS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    }
};

const ADC_Config ADC_config[CC1310_LAUNCHXL_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCDCOUPL], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCDCOUPL]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCVSS], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCVSS]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCVDDS], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCVDDS]},
};

const uint_least8_t ADC_count = CC1310_LAUNCHXL_ADCCOUNT;

/*
 *  =============================== Crypto ===============================
 */
#include <ti/drivers/crypto/CryptoCC26XX.h>

CryptoCC26XX_Object cryptoCC26XXObjects[CC1310_LAUNCHXL_CRYPTOCOUNT];

const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC1310_LAUNCHXL_CRYPTOCOUNT] = {
    {
        .baseAddr       = CRYPTO_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_CRYPTO,
        .intNum         = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority    = ~0,
    }
};

const CryptoCC26XX_Config CryptoCC26XX_config[CC1310_LAUNCHXL_CRYPTOCOUNT] = {
    {
         .object  = &cryptoCC26XXObjects[CC1310_LAUNCHXL_CRYPTO0],
         .hwAttrs = &cryptoCC26XXHWAttrs[CC1310_LAUNCHXL_CRYPTO0]
    }
};

/*
 *  =============================== AESCCM ===============================
 */
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

AESCCMCC26XX_Object aesccmCC26XXObjects[CC1310_LAUNCHXL_AESCCMCOUNT];

const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CC1310_LAUNCHXL_AESCCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCCM_Config AESCCM_config[CC1310_LAUNCHXL_AESCCMCOUNT] = {
    {
         .object  = &aesccmCC26XXObjects[CC1310_LAUNCHXL_AESCCM0],
         .hwAttrs = &aesccmCC26XXHWAttrs[CC1310_LAUNCHXL_AESCCM0]
    },
};

const uint_least8_t AESCCM_count = CC1310_LAUNCHXL_AESCCMCOUNT;


/*
 *  =============================== AESGCM ===============================
 */
#include <ti/drivers/AESGCM.h>
#include <ti/drivers/aesgcm/AESGCMCC26XX.h>

AESGCMCC26XX_Object aesgcmCC26XXObjects[CC1310_LAUNCHXL_AESGCMCOUNT];

const AESGCMCC26XX_HWAttrs aesgcmCC26XXHWAttrs[CC1310_LAUNCHXL_AESGCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESGCM_Config AESGCM_config[CC1310_LAUNCHXL_AESGCMCOUNT] = {
    {
         .object  = &aesgcmCC26XXObjects[CC1310_LAUNCHXL_AESGCM0],
         .hwAttrs = &aesgcmCC26XXHWAttrs[CC1310_LAUNCHXL_AESGCM0]
    },
};

const uint_least8_t AESGCM_count = CC1310_LAUNCHXL_AESGCMCOUNT;

/*
 *  =============================== AESCBC ===============================
 */
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>

AESCBCCC26XX_Object aescbcCC26XXObjects[CC1310_LAUNCHXL_AESCBCCOUNT];

const AESCBCCC26XX_HWAttrs aescbcCC26XXHWAttrs[CC1310_LAUNCHXL_AESCBCCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCBC_Config AESCBC_config[CC1310_LAUNCHXL_AESCBCCOUNT] = {
    {
         .object  = &aescbcCC26XXObjects[CC1310_LAUNCHXL_AESCBC0],
         .hwAttrs = &aescbcCC26XXHWAttrs[CC1310_LAUNCHXL_AESCBC0]
    },
};

const uint_least8_t AESCBC_count = CC1310_LAUNCHXL_AESCBCCOUNT;

/*
 *  =============================== AESCTR ===============================
 */
#include <ti/drivers/AESCTR.h>
#include <ti/drivers/aesctr/AESCTRCC26XX.h>

AESCTRCC26XX_Object aesctrCC26XXObjects[CC1310_LAUNCHXL_AESCTRCOUNT];

const AESCTRCC26XX_HWAttrs aesctrCC26XXHWAttrs[CC1310_LAUNCHXL_AESCTRCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCTR_Config AESCTR_config[CC1310_LAUNCHXL_AESCTRCOUNT] = {
    {
         .object  = &aesctrCC26XXObjects[CC1310_LAUNCHXL_AESCTR0],
         .hwAttrs = &aesctrCC26XXHWAttrs[CC1310_LAUNCHXL_AESCTR0]
    },
};

const uint_least8_t AESCTR_count = CC1310_LAUNCHXL_AESCTRCOUNT;

/*
 *  =============================== AESECB ===============================
 */
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

AESECBCC26XX_Object aesecbCC26XXObjects[CC1310_LAUNCHXL_AESECBCOUNT];

const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CC1310_LAUNCHXL_AESECBCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESECB_Config AESECB_config[CC1310_LAUNCHXL_AESECBCOUNT] = {
    {
         .object  = &aesecbCC26XXObjects[CC1310_LAUNCHXL_AESECB0],
         .hwAttrs = &aesecbCC26XXHWAttrs[CC1310_LAUNCHXL_AESECB0]
    },
};

const uint_least8_t AESECB_count = CC1310_LAUNCHXL_AESECBCOUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */
#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

AESCTRDRBGXX_Object aesctrdrbgXXObjects[CC1310_LAUNCHXL_AESCTRDRBGCOUNT];

const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CC1310_LAUNCHXL_AESCTRDRBGCOUNT] = {
    {
        .aesctrIndex       = CC1310_LAUNCHXL_AESCTR0,
    }
};

const AESCTRDRBG_Config AESCTRDRBG_config[CC1310_LAUNCHXL_AESCTRDRBGCOUNT] = {
    {
         .object  = &aesctrdrbgXXObjects[CC1310_LAUNCHXL_AESCTRDRBG0],
         .hwAttrs = &aesctrdrbgXXHWAttrs[CC1310_LAUNCHXL_AESCTRDRBG0]
    },
};

const uint_least8_t AESCTRDRBG_count = CC1310_LAUNCHXL_AESCTRDRBGCOUNT;

/*
 *  =============================== TRNG ===============================
 */
#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26XX.h>

TRNGCC26XX_Object trngCC26XXObjects[CC1310_LAUNCHXL_TRNGCOUNT];

const TRNGCC26XX_HWAttrs trngCC26X2HWAttrs[CC1310_LAUNCHXL_TRNGCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .samplesPerCycle   = 240000,
    }
};

const TRNG_Config TRNG_config[CC1310_LAUNCHXL_TRNGCOUNT] = {
    {
         .object  = &trngCC26XXObjects[CC1310_LAUNCHXL_TRNG0],
         .hwAttrs = &trngCC26X2HWAttrs[CC1310_LAUNCHXL_TRNG0]
    },
};

const uint_least8_t TRNG_count = CC1310_LAUNCHXL_TRNGCOUNT;


/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1310_LAUNCHXL.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array. Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input pins */
   GPIOCC26XX_DIO_01 | GPIO_DO_NOT_CONFIG,  /* CONFIG_GPIO_EPD_BUSY  */
   /* Output pins */
   GPIOCC26XX_DIO_00 | GPIO_DO_NOT_CONFIG,  /* CONFIG_GPIO_EPD_PWR */
   GPIOCC26XX_DIO_02 | GPIO_DO_NOT_CONFIG,  /* CONFIG_GPIO_EPD_CS */
   GPIOCC26XX_DIO_03 | GPIO_DO_NOT_CONFIG,  /* CONFIG_GPIO_EPD_CLK */
   GPIOCC26XX_DIO_04 | GPIO_DO_NOT_CONFIG,  /* CONFIG_GPIO_EPD_SDI */
   GPIOCC26XX_DIO_05 | GPIO_DO_NOT_CONFIG,  /* CONFIG_GPIO_EPD_RST */
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1310_LAUNCH.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /* CONFIG_GPIO_EPD_BUSY */
};

const GPIOCC26XX_Config GPIOCC26XX_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = CC1310_LAUNCHXL_GPIOCOUNT,
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[CC1310_LAUNCHXL_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER0], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER0], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER0B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER1], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER1], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER2], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER2], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER3], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER3], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER3B], GPT_B },
};

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

const PIN_Config BoardGpioInitTable[] = {

    CHROMA_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,  /* External flash chip select */
    CHROMA_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* UART RX via debugger back channel */
    CHROMA_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,                         /* UART TX via debugger back channel */
    CHROMA_SPI0_MOSI | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,     /* SPI master out - slave in */
    CHROMA_SPI0_MISO | PIN_INPUT_EN | PIN_PULLUP,                                            /* SPI master in - slave out */
    CHROMA_SPI0_CLK | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,      /* SPI clock */

// CONFIG_GPIO_EPD_BUSY
   IOID_1 | PIN_INPUT_EN | PIN_PULLDOWN,

// CONFIG_GPIO_EPD_PWR
   IOID_0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED, 
// CONFIG_GPIO_EPD_CS
   IOID_2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,  
// CONFIG_GPIO_EPD_CLK
   IOID_3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED, 
// CONFIG_GPIO_EPD_SDI 
   IOID_4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,
// CONFIG_GPIO_EPD_RST
   IOID_5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,

    PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = &PowerCC26XX_standbyPolicy,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = true,
    .calibrateRCOSC_LF  = true,
    .calibrateRCOSC_HF  = true,
};


/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,       /* Lowest HWI priority */
    .swiPriority        = 0,        /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,     /* Keep XOSC dependency while in standby */
    .globalCallback     = NULL,     /* No board specific callback */
    .globalEventMask    = 0         /* No events subscribed to */
};

/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

SPICC26XXDMA_Object spiCC26XXDMAObjects[CC1310_LAUNCHXL_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC1310_LAUNCHXL_SPICOUNT] = {
    {
        .baseAddr           = SSI0_BASE,
        .intNum             = INT_SSI0_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin            = CHROMA_SPI0_MOSI,
        .misoPin            = CHROMA_SPI0_MISO,
        .clkPin             = CHROMA_SPI0_CLK,
        .csnPin             = CHROMA_SPI0_CSN,
        .minDmaTransferSize = 10
    }
};

const SPI_Config SPI_config[CC1310_LAUNCHXL_SPICOUNT] = {
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CHROMA_SPI0],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CHROMA_SPI0]
    }
};

const uint_least8_t SPI_count = CC1310_LAUNCHXL_SPICOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects[CHROMA_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CHROMA_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CHROMA_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = CHROMA_UART_TX,
        .rxPin          = CHROMA_UART_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[CHROMA_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[CHROMA_UART0]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    }
};

const UART_Config UART_config[CHROMA_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CHROMA_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[CHROMA_UART0]
    },
};

const uint_least8_t UART_count = CHROMA_UARTCOUNT;

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC1310_LAUNCHXL_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC1310_LAUNCHXL_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC1310_LAUNCHXL_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC1310_LAUNCHXL_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC1310_LAUNCHXL_UDMA0]
    },
};

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC1310_LAUNCHXL_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC1310_LAUNCHXL_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 1000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[CC1310_LAUNCHXL_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[CC1310_LAUNCHXL_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[CC1310_LAUNCHXL_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = CC1310_LAUNCHXL_WATCHDOGCOUNT;

/*
 *  Board-specific initialization function to disable external flash.
 *  This function is defined in the file CC1310_LAUNCHXL_fxns.c
 */
extern void Board_initHook(void);

/*
 *  ======== CC1310_LAUNCHXL_initGeneral ========
 */
void CC1310_LAUNCHXL_initGeneral(void)
{
    Power_init();

    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }

    /* Perform board-specific initialization */
    Board_initHook();
}

/*
 *  ======== Board_init ========
 */
void Board_init(void)
{
    CC1310_LAUNCHXL_initGeneral();
}
