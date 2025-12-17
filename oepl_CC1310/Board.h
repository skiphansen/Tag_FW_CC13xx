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

#ifndef __BOARD_H
#define __BOARD_H

#define Board_CC1310_LAUNCHXL

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Board.h>

#define Board_initGeneral()     Board_init()  /* deprecated */

#include "Chroma_CC1310.h"

#define Board_shutDownExtFlash() OEPL_CC1310_shutDownExtFlash()
#define Board_wakeUpExtFlash() OEPL_CC1310_wakeUpExtFlash()


/* These #defines allow us to reuse TI-RTOS across other device families */

#define Board_ADC0              OEPL_CC1310_ADC0
#define Board_ADC1              OEPL_CC1310_ADC1

#define Board_ADCBUF0           OEPL_CC1310_ADCBUF0
#define Board_ADCBUF0CHANNEL0   OEPL_CC1310_ADCBUF0CHANNEL0
#define Board_ADCBUF0CHANNEL1   OEPL_CC1310_ADCBUF0CHANNEL1

#define Board_CRYPTO0           OEPL_CC1310_CRYPTO0
#define Board_AESCCM0           OEPL_CC1310_AESCCM0
#define Board_AESGCM0           OEPL_CC1310_AESGCM0
#define Board_AESCBC0           OEPL_CC1310_AESCBC0
#define Board_AESCTR0           OEPL_CC1310_AESCTR0
#define Board_AESECB0           OEPL_CC1310_AESECB0
#define Board_AESCTRDRBG0       OEPL_CC1310_AESCTRDRBG0
#define Board_TRNG0             OEPL_CC1310_TRNG0

#define Board_DIO0              OEPL_CC1310_DIO0
#define Board_DIO1              OEPL_CC1310_DIO1
#define Board_DIO12             OEPL_CC1310_DIO12
#define Board_DIO15             OEPL_CC1310_DIO15
#define Board_DIO16_TDO         OEPL_CC1310_DIO16_TDO
#define Board_DIO17_TDI         OEPL_CC1310_DIO17_TDI
#define Board_DIO21             OEPL_CC1310_DIO21
#define Board_DIO22             OEPL_CC1310_DIO22

#define Board_GPIO_BUTTON0      OEPL_CC1310_GPIO_S1
#define Board_GPIO_BUTTON1      OEPL_CC1310_GPIO_S2
#define Board_GPIO_BTN1         OEPL_CC1310_GPIO_S1
#define Board_GPIO_BTN2         OEPL_CC1310_GPIO_S2
#define Board_GPIO_LED0         OEPL_CC1310_GPIO_LED_RED
#define Board_GPIO_LED1         OEPL_CC1310_GPIO_LED_GREEN
#define Board_GPIO_RLED         OEPL_CC1310_GPIO_LED_RED
#define Board_GPIO_GLED         OEPL_CC1310_GPIO_LED_GREEN
#define Board_GPIO_LED_ON       OEPL_CC1310_GPIO_LED_ON
#define Board_GPIO_LED_OFF      OEPL_CC1310_GPIO_LED_OFF
#define Board_GPIO_TMP116_EN    OEPL_CC1310_GPIO_TMP116_EN

#define Board_GPTIMER0A         OEPL_CC1310_GPTIMER0A
#define Board_GPTIMER0B         OEPL_CC1310_GPTIMER0B
#define Board_GPTIMER1A         OEPL_CC1310_GPTIMER1A
#define Board_GPTIMER1B         OEPL_CC1310_GPTIMER1B
#define Board_GPTIMER2A         OEPL_CC1310_GPTIMER2A
#define Board_GPTIMER2B         OEPL_CC1310_GPTIMER2B
#define Board_GPTIMER3A         OEPL_CC1310_GPTIMER3A
#define Board_GPTIMER3B         OEPL_CC1310_GPTIMER3B

#define Board_I2C0              OEPL_CC1310_I2C0
#define Board_I2C_TMP           OEPL_CC1310_I2C0

#define Board_I2S0              OEPL_CC1310_I2S0
#define Board_I2S_ADO           OEPL_CC1310_I2S_ADO
#define Board_I2S_ADI           OEPL_CC1310_I2S_ADI
#define Board_I2S_BCLK          OEPL_CC1310_I2S_BCLK
#define Board_I2S_MCLK          OEPL_CC1310_I2S_MCLK
#define Board_I2S_WCLK          OEPL_CC1310_I2S_WCLK

#define Board_NVSINTERNAL       OEPL_CC1310_NVSCC26XX0
#define Board_NVSEXTERNAL       OEPL_CC1310_NVSSPI25X0

#define Board_PIN_BUTTON0       OEPL_CC1310_PIN_BTN1
#define Board_PIN_BUTTON1       OEPL_CC1310_PIN_BTN2
#define Board_PIN_BTN1          OEPL_CC1310_PIN_BTN1
#define Board_PIN_BTN2          OEPL_CC1310_PIN_BTN2
#define Board_PIN_LED0          OEPL_CC1310_PIN_RLED
#define Board_PIN_LED1          OEPL_CC1310_PIN_GLED
#define Board_PIN_LED2          OEPL_CC1310_PIN_RLED
#define Board_PIN_RLED          OEPL_CC1310_PIN_RLED
#define Board_PIN_GLED          OEPL_CC1310_PIN_GLED

#define Board_PWM0              OEPL_CC1310_PWM0
#define Board_PWM1              OEPL_CC1310_PWM1
#define Board_PWM2              OEPL_CC1310_PWM2
#define Board_PWM3              OEPL_CC1310_PWM3
#define Board_PWM4              OEPL_CC1310_PWM4
#define Board_PWM5              OEPL_CC1310_PWM5
#define Board_PWM6              OEPL_CC1310_PWM6
#define Board_PWM7              OEPL_CC1310_PWM7

#define Board_SD0               OEPL_CC1310_SDSPI0

#define Board_SPI0              OEPL_CC1310_SPI0
#define Board_SPI1              OEPL_CC1310_SPI1
#define Board_SPI_FLASH_CS      OEPL_CC1310_SPI_FLASH_CS
#define Board_FLASH_CS_ON       0
#define Board_FLASH_CS_OFF      1

#define Board_SPI_MASTER        OEPL_CC1310_SPI0
#define Board_SPI_SLAVE         OEPL_CC1310_SPI0
#define Board_SPI_MASTER_READY  OEPL_CC1310_SPI_MASTER_READY
#define Board_SPI_SLAVE_READY   OEPL_CC1310_SPI_SLAVE_READY

#define Board_UART0             CHROMA_UART0

#define Board_WATCHDOG0         OEPL_CC1310_WATCHDOG0

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
