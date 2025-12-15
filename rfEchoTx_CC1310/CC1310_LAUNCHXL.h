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
/** ============================================================================
 *  @file       CC1310_LAUNCHXL.h
 *
 *  @brief      CC1310 LaunchPad Board Specific header file.
 *
 *  The CC1310_LAUNCHXL header file should be included in an application as
 *  follows:
 *  @code
 *  #include "CC1310_LAUNCHXL.h"
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __CC1310_LAUNCHXL_BOARD_H__
#define __CC1310_LAUNCHXL_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
/* SPI */
#define CHROMA_SPI_FLASH_CS          IOID_11 // modified for Chroma 21

/* SPI Board */
#define CHROMA_SPI0_MISO             IOID_10          /* RF1.20 */
#define CHROMA_SPI0_MOSI             IOID_8          /* modified for Chroma 21 */
#define CHROMA_SPI0_CLK              IOID_9         /* RF1.16 */
#define CHROMA_SPI0_CSN              PIN_UNASSIGNED
#define CHROMA_EPD_BUSY              IOID_1
#define CHROMA_EPD_PWR               IOID_0
#define CHROMA_EPD_CS                IOID_2
#define CHROMA_EPD_CLK               IOID_3
#define CHROMA_EPD_SDI               IOID_4
#define CHROMA_EPD_RST               IOID_5

/* UART Board */
#define CHROMA_UART_RX               IOID_6          /* RXD modified for Chroma21 */
#define CHROMA_UART_TX               IOID_7          /* TXD modified for Chroma21 */
#define CHROMA_UART_CTS              PIN_UNASSIGNED   /* CTS */
#define CHROMA_UART_RTS              PIN_UNASSIGNED   /* RTS */

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC1310_LAUNCHXL_initGeneral(void);

/*!
 *  @brief  Turn off the external flash on LaunchPads
 *
 */
void CC1310_LAUNCHXL_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void CC1310_LAUNCHXL_wakeUpExtFlash(void);

/*!
 *  @def    CC1310_LAUNCHXL_ADCBufName
 *  @brief  Enum of ADCBufs
 */
typedef enum CC1310_LAUNCHXL_ADCBufName {
    CC1310_LAUNCHXL_ADCBUF0 = 0,

    CC1310_LAUNCHXL_ADCBUFCOUNT
} CC1310_LAUNCHXL_ADCBufName;

/*!
 *  @def    CC1310_LAUNCHXL_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC1310_LAUNCHXL_ADCBuf0ChannelName {
    CC1310_LAUNCHXL_ADCBUF0CHANNELVDDS,
    CC1310_LAUNCHXL_ADCBUF0CHANNELDCOUPL,
    CC1310_LAUNCHXL_ADCBUF0CHANNELVSS,

    CC1310_LAUNCHXL_ADCBUF0CHANNELCOUNT
} CC1310_LAUNCHXL_ADCBuf0ChannelName;

/*!
 *  @def    CC1310_LAUNCHXL_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC1310_LAUNCHXL_ADCName {
    CC1310_LAUNCHXL_ADCDCOUPL,
    CC1310_LAUNCHXL_ADCVSS,
    CC1310_LAUNCHXL_ADCVDDS,

    CC1310_LAUNCHXL_ADCCOUNT
} CC1310_LAUNCHXL_ADCName;

/*!
 *  @def    CC1310_LAUNCHXL_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC1310_LAUNCHXL_CryptoName {
    CC1310_LAUNCHXL_CRYPTO0 = 0,

    CC1310_LAUNCHXL_CRYPTOCOUNT
} CC1310_LAUNCHXL_CryptoName;

/*!
 *  @def    CC1310_LAUNCHXL_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC1310_LAUNCHXL_AESCCMName {
    CC1310_LAUNCHXL_AESCCM0 = 0,

    CC1310_LAUNCHXL_AESCCMCOUNT
} CC1310_LAUNCHXL_AESCCMName;

/*!
 *  @def    CC1310_LAUNCHXL_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC1310_LAUNCHXL_AESGCMName {
    CC1310_LAUNCHXL_AESGCM0 = 0,

    CC1310_LAUNCHXL_AESGCMCOUNT
} CC1310_LAUNCHXL_AESGCMName;

/*!
 *  @def    CC1310_LAUNCHXL_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC1310_LAUNCHXL_AESCBCName {
    CC1310_LAUNCHXL_AESCBC0 = 0,

    CC1310_LAUNCHXL_AESCBCCOUNT
} CC1310_LAUNCHXL_AESCBCName;

/*!
 *  @def    CC1310_LAUNCHXL_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC1310_LAUNCHXL_AESCTRName {
    CC1310_LAUNCHXL_AESCTR0 = 0,

    CC1310_LAUNCHXL_AESCTRCOUNT
} CC1310_LAUNCHXL_AESCTRName;

/*!
 *  @def    CC1310_LAUNCHXL_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC1310_LAUNCHXL_AESECBName {
    CC1310_LAUNCHXL_AESECB0 = 0,

    CC1310_LAUNCHXL_AESECBCOUNT
} CC1310_LAUNCHXL_AESECBName;

/*!
 *  @def    CC1310_LAUNCHXL_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC1310_LAUNCHXL_AESCTRDRBGName {
    CC1310_LAUNCHXL_AESCTRDRBG0 = 0,

    CC1310_LAUNCHXL_AESCTRDRBGCOUNT
} CC1310_LAUNCHXL_AESCTRDRBGName;

/*!
 *  @def    CC1310_LAUNCHXL_TRNGName
 *  @brief  Enum of TRNG names
 */
typedef enum CC1310_LAUNCHXL_TRNGName {
    CC1310_LAUNCHXL_TRNG0 = 0,

    CC1310_LAUNCHXL_TRNGCOUNT
} CC1310_LAUNCHXL_TRNGName;

/*!
 *  @def    CC1310_CHROMAL_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC1310_CHROMAL_GPIOName {
    CONFIG_GPIO_EPD_BUSY,
    /* Output pins */
    CONFIG_GPIO_EPD_PWR,
    CONFIG_GPIO_EPD_CS,
    CONFIG_GPIO_EPD_CLK,
    CONFIG_GPIO_EPD_SDI,
    CONFIG_GPIO_EPD_RST,
    CC1310_LAUNCHXL_GPIOCOUNT
} CC1310_CHROMAL_GPIOName;

/*!
 *  @def    CC1310_LAUNCHXL_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC1310_LAUNCHXL_GPTimerName {
    CC1310_LAUNCHXL_GPTIMER0A = 0,
    CC1310_LAUNCHXL_GPTIMER0B,
    CC1310_LAUNCHXL_GPTIMER1A,
    CC1310_LAUNCHXL_GPTIMER1B,
    CC1310_LAUNCHXL_GPTIMER2A,
    CC1310_LAUNCHXL_GPTIMER2B,
    CC1310_LAUNCHXL_GPTIMER3A,
    CC1310_LAUNCHXL_GPTIMER3B,

    CC1310_LAUNCHXL_GPTIMERPARTSCOUNT
} CC1310_LAUNCHXL_GPTimerName;

/*!
 *  @def    CC1310_LAUNCHXL_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC1310_LAUNCHXL_GPTimers {
    CC1310_LAUNCHXL_GPTIMER0 = 0,
    CC1310_LAUNCHXL_GPTIMER1,
    CC1310_LAUNCHXL_GPTIMER2,
    CC1310_LAUNCHXL_GPTIMER3,

    CC1310_LAUNCHXL_GPTIMERCOUNT
} CC1310_LAUNCHXL_GPTimers;

/*!
 *  @def    CC1310_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC1310_LAUNCHXL_I2CName {
    CC1310_LAUNCHXL_I2CCOUNT
} CC1310_LAUNCHXL_I2CName;

/*!
 *  @def    CC1310_LAUNCHXL_I2SName
 *  @brief  Enum of I2S names
 */
typedef enum CC1310_LAUNCHXL_I2SName {
    CC1310_LAUNCHXL_I2S0 = 0,

    CC1310_LAUNCHXL_I2SCOUNT
} CC1310_LAUNCHXL_I2SName;

/*!
 *  @def    CC1310_LAUNCHXL_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC1310_LAUNCHXL_NVSName {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    CC1310_LAUNCHXL_NVSCC26XX0 = 0,
#endif
#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH
    CC1310_LAUNCHXL_NVSSPI25X0,
#endif

    CC1310_LAUNCHXL_NVSCOUNT
} CC1310_LAUNCHXL_NVSName;

/*!
 *  @def    CC1310_LAUNCHXL_PWM
 *  @brief  Enum of PWM outputs
 */
typedef enum CC1310_LAUNCHXL_PWMName {
    CC1310_LAUNCHXL_PWM0 = 0,
    CC1310_LAUNCHXL_PWM1,
    CC1310_LAUNCHXL_PWM2,
    CC1310_LAUNCHXL_PWM3,
    CC1310_LAUNCHXL_PWM4,
    CC1310_LAUNCHXL_PWM5,
    CC1310_LAUNCHXL_PWM6,
    CC1310_LAUNCHXL_PWM7,

    CC1310_LAUNCHXL_PWMCOUNT
} CC1310_LAUNCHXL_PWMName;

/*!
 *  @def    CC1310_LAUNCHXL_SDName
 *  @brief  Enum of SD names
 */
typedef enum CC1310_LAUNCHXL_SDName {
    CC1310_LAUNCHXL_SDSPI0 = 0,

    CC1310_LAUNCHXL_SDCOUNT
} CC1310_LAUNCHXL_SDName;

/*!
 *  @def    CC1310_LAUNCHXL_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC1310_LAUNCHXL_SPIName {
    CHROMA_SPI0 = 0,
    CC1310_LAUNCHXL_SPICOUNT
} CC1310_LAUNCHXL_SPIName;

/*!
 *  @def    CHROMA_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CHROMA_UARTName {
    CHROMA_UART0 = 0,

    CHROMA_UARTCOUNT
} CHROMA_UARTName;

/*!
 *  @def    CC1310_LAUNCHXL_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1310_LAUNCHXL_UDMAName {
    CC1310_LAUNCHXL_UDMA0 = 0,

    CC1310_LAUNCHXL_UDMACOUNT
} CC1310_LAUNCHXL_UDMAName;

/*!
 *  @def    CC1310_LAUNCHXL_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC1310_LAUNCHXL_WatchdogName {
    CC1310_LAUNCHXL_WATCHDOG0 = 0,

    CC1310_LAUNCHXL_WATCHDOGCOUNT
} CC1310_LAUNCHXL_WatchdogName;


#ifdef __cplusplus
}
#endif

#endif /* __CC1310_LAUNCHXL_BOARD_H__ */
