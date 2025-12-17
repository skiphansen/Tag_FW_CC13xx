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
 *  @file       OEPL_CC1310.h
 *
 *  @brief      CC1310 LaunchPad Board Specific header file.
 *
 *  The OEPL_CC1310 header file should be included in an application as
 *  follows:
 *  @code
 *  #include "OEPL_CC1310.h"
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __OEPL_CC1310_BOARD_H__
#define __OEPL_CC1310_BOARD_H__

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
void OEPL_CC1310_initGeneral(void);

/*!
 *  @brief  Turn off the external flash on LaunchPads
 *
 */
void OEPL_CC1310_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void OEPL_CC1310_wakeUpExtFlash(void);

/*!
 *  @def    OEPL_CC1310_ADCBufName
 *  @brief  Enum of ADCBufs
 */
typedef enum OEPL_CC1310_ADCBufName {
    OEPL_CC1310_ADCBUF0 = 0,

    OEPL_CC1310_ADCBUFCOUNT
} OEPL_CC1310_ADCBufName;

/*!
 *  @def    OEPL_CC1310_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum OEPL_CC1310_ADCBuf0ChannelName {
    OEPL_CC1310_ADCBUF0CHANNELVDDS,
    OEPL_CC1310_ADCBUF0CHANNELDCOUPL,
    OEPL_CC1310_ADCBUF0CHANNELVSS,

    OEPL_CC1310_ADCBUF0CHANNELCOUNT
} OEPL_CC1310_ADCBuf0ChannelName;

/*!
 *  @def    OEPL_CC1310_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum OEPL_CC1310_ADCName {
    OEPL_CC1310_ADCDCOUPL,
    OEPL_CC1310_ADCVSS,
    OEPL_CC1310_ADCVDDS,

    OEPL_CC1310_ADCCOUNT
} OEPL_CC1310_ADCName;

/*!
 *  @def    OEPL_CC1310_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum OEPL_CC1310_CryptoName {
    OEPL_CC1310_CRYPTO0 = 0,

    OEPL_CC1310_CRYPTOCOUNT
} OEPL_CC1310_CryptoName;

/*!
 *  @def    OEPL_CC1310_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum OEPL_CC1310_AESCCMName {
    OEPL_CC1310_AESCCM0 = 0,

    OEPL_CC1310_AESCCMCOUNT
} OEPL_CC1310_AESCCMName;

/*!
 *  @def    OEPL_CC1310_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum OEPL_CC1310_AESGCMName {
    OEPL_CC1310_AESGCM0 = 0,

    OEPL_CC1310_AESGCMCOUNT
} OEPL_CC1310_AESGCMName;

/*!
 *  @def    OEPL_CC1310_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum OEPL_CC1310_AESCBCName {
    OEPL_CC1310_AESCBC0 = 0,

    OEPL_CC1310_AESCBCCOUNT
} OEPL_CC1310_AESCBCName;

/*!
 *  @def    OEPL_CC1310_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum OEPL_CC1310_AESCTRName {
    OEPL_CC1310_AESCTR0 = 0,

    OEPL_CC1310_AESCTRCOUNT
} OEPL_CC1310_AESCTRName;

/*!
 *  @def    OEPL_CC1310_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum OEPL_CC1310_AESECBName {
    OEPL_CC1310_AESECB0 = 0,

    OEPL_CC1310_AESECBCOUNT
} OEPL_CC1310_AESECBName;

/*!
 *  @def    OEPL_CC1310_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum OEPL_CC1310_AESCTRDRBGName {
    OEPL_CC1310_AESCTRDRBG0 = 0,

    OEPL_CC1310_AESCTRDRBGCOUNT
} OEPL_CC1310_AESCTRDRBGName;

/*!
 *  @def    OEPL_CC1310_TRNGName
 *  @brief  Enum of TRNG names
 */
typedef enum OEPL_CC1310_TRNGName {
    OEPL_CC1310_TRNG0 = 0,

    OEPL_CC1310_TRNGCOUNT
} OEPL_CC1310_TRNGName;

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
    OEPL_CC1310_GPIOCOUNT
} CC1310_CHROMAL_GPIOName;

/*!
 *  @def    OEPL_CC1310_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum OEPL_CC1310_GPTimerName {
    OEPL_CC1310_GPTIMER0A = 0,
    OEPL_CC1310_GPTIMER0B,
    OEPL_CC1310_GPTIMER1A,
    OEPL_CC1310_GPTIMER1B,
    OEPL_CC1310_GPTIMER2A,
    OEPL_CC1310_GPTIMER2B,
    OEPL_CC1310_GPTIMER3A,
    OEPL_CC1310_GPTIMER3B,

    OEPL_CC1310_GPTIMERPARTSCOUNT
} OEPL_CC1310_GPTimerName;

/*!
 *  @def    OEPL_CC1310_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum OEPL_CC1310_GPTimers {
    OEPL_CC1310_GPTIMER0 = 0,
    OEPL_CC1310_GPTIMER1,
    OEPL_CC1310_GPTIMER2,
    OEPL_CC1310_GPTIMER3,

    OEPL_CC1310_GPTIMERCOUNT
} OEPL_CC1310_GPTimers;

/*!
 *  @def    OEPL_CC1310_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum OEPL_CC1310_I2CName {
    OEPL_CC1310_I2CCOUNT
} OEPL_CC1310_I2CName;

/*!
 *  @def    OEPL_CC1310_I2SName
 *  @brief  Enum of I2S names
 */
typedef enum OEPL_CC1310_I2SName {
    OEPL_CC1310_I2S0 = 0,

    OEPL_CC1310_I2SCOUNT
} OEPL_CC1310_I2SName;

/*!
 *  @def    OEPL_CC1310_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum OEPL_CC1310_NVSName {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    OEPL_CC1310_NVSCC26XX0 = 0,
#endif
#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH
    OEPL_CC1310_NVSSPI25X0,
#endif

    OEPL_CC1310_NVSCOUNT
} OEPL_CC1310_NVSName;

/*!
 *  @def    OEPL_CC1310_PWM
 *  @brief  Enum of PWM outputs
 */
typedef enum OEPL_CC1310_PWMName {
    OEPL_CC1310_PWM0 = 0,
    OEPL_CC1310_PWM1,
    OEPL_CC1310_PWM2,
    OEPL_CC1310_PWM3,
    OEPL_CC1310_PWM4,
    OEPL_CC1310_PWM5,
    OEPL_CC1310_PWM6,
    OEPL_CC1310_PWM7,

    OEPL_CC1310_PWMCOUNT
} OEPL_CC1310_PWMName;

/*!
 *  @def    OEPL_CC1310_SDName
 *  @brief  Enum of SD names
 */
typedef enum OEPL_CC1310_SDName {
    OEPL_CC1310_SDSPI0 = 0,

    OEPL_CC1310_SDCOUNT
} OEPL_CC1310_SDName;

/*!
 *  @def    OEPL_CC1310_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum OEPL_CC1310_SPIName {
    CHROMA_SPI0 = 0,
    OEPL_CC1310_SPICOUNT
} OEPL_CC1310_SPIName;

/*!
 *  @def    CHROMA_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CHROMA_UARTName {
    CHROMA_UART0 = 0,

    CHROMA_UARTCOUNT
} CHROMA_UARTName;

/*!
 *  @def    OEPL_CC1310_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum OEPL_CC1310_UDMAName {
    OEPL_CC1310_UDMA0 = 0,

    OEPL_CC1310_UDMACOUNT
} OEPL_CC1310_UDMAName;

/*!
 *  @def    OEPL_CC1310_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum OEPL_CC1310_WatchdogName {
    OEPL_CC1310_WATCHDOG0 = 0,

    OEPL_CC1310_WATCHDOGCOUNT
} OEPL_CC1310_WatchdogName;


#ifdef __cplusplus
}
#endif

#endif /* __OEPL_CC1310_BOARD_H__ */
