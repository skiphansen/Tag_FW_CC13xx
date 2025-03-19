#ifndef EPD_H
#define EPD_H

#include <stdint.h>
#include "ti_drivers_config.h"

// Pin Definitions
#define RST_PIN         CONFIG_GPIO_EPD_RST
#define BUSY_PIN        CONFIG_GPIO_EPD_BUSY
#define PWR_PIN         CONFIG_GPIO_EPD_PWR
#define EPD_WIDTH       800
#define EPD_HEIGHT      480

// Function Prototypes
void Epd_Init(void);
void Epd_Reset(void);
void Epd_WaitUntilIdle(void);
void Epd_SendCommand(uint8_t command);
void Epd_SendData(uint8_t data);
void Epd_Sleep(void);
void Epd_DigitalWrite(int pin, int value);
int Epd_DigitalRead(int pin);
void Epd_DelayMs(unsigned int delaytime);
void Epd_TurnOnDisplay(void);
void Epd_ClearBlack(void);
void Epd_Draw(void);
void Epd_DrawPattern(void);

#endif // EPD_H
