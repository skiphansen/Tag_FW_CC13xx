#include <ti/drivers/GPIO.h>
#include "cc13xx.h"

#if defined(DeviceFamily_CC13X0)
#include "Board.h"
#endif

#if defined(DeviceFamily_CC13X1)
#include "ti_drivers_config.h"
#endif

#include "logging.h"

#ifndef __OEPL_OELP__
#define __OEPL_OELP__
#define PROGMEM
#define memcpy_P memcpy
#define pgm_read_byte(a) (*(uint8_t *)a)
#define pgm_read_word(a) (*(uint16_t *)a)
#define pgm_read_dword(a) (*(uint32_t *)a)
#define LOW 0
#define HIGH 1
#ifndef I2C_SLAVE
#define I2C_SLAVE 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#endif
// forward references
void bbepWakeUp(BBEPDISP *pBBEP);
void write9bits(uint16_t data);

void bbepInitIO(BBEPDISP *pBBEP, uint32_t u32Speed)
{
   pBBEP->iRSTPin = CONFIG_GPIO_EPD_RST;
   pBBEP->iBUSYPin = CONFIG_GPIO_EPD_BUSY;
}

static void digitalWrite(int iPin, int iState) 
{
   GPIO_write(iPin,iState);
}

static int digitalRead(int iPin)
{
   return GPIO_read(iPin);
}

static void delay(int iTime)
{
   WaitMs(iTime);
}

void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
#if 0
   LOG("Writing %d bytes of data:\n",iLen);
   DumpHex(pData,iLen);
#endif

   GPIO_write(CONFIG_GPIO_EPD_CS,0);
   while(iLen--) {
      write9bits((1 << 8) | *pData++);
   }
   GPIO_write(CONFIG_GPIO_EPD_CS,1);
}
//
// Convenience function to write a command byte along with a data
// byte (it's single parameter)
//
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2)
{
    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    LOG("Cmd 0x%x, Data 0x%x\n",cmd1,cmd2);
    GPIO_write(CONFIG_GPIO_EPD_CS,0);
    write9bits(cmd1);
    write9bits((1 << 8) | cmd2);
    GPIO_write(CONFIG_GPIO_EPD_CS,1);
}

// 
// Write a single byte as a COMMAND (D/C set low)
// 
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd)
{
    if (!pBBEP->is_awake) { 
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    GPIO_write(CONFIG_GPIO_EPD_CS,0);
    write9bits(cmd);
    GPIO_write(CONFIG_GPIO_EPD_CS,1);
}

long millis(void)
{
  return clock_time() / 100;
}

long micros(void)
{
  return clock_time() * 10;
}

void bbepSetCS2(BBEPDISP *pBBEP, uint8_t cs)
{
    pBBEP->iCS1Pin = pBBEP->iCSPin;
    pBBEP->iCS2Pin = cs;
#if 0
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH); // disable second CS for now
#endif
}



#endif // __OEPL_OELP__
