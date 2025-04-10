#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include "epd.h"
#include "img.h"
#include "main.h"
#include "oepl-proto.h"
#include "oepl-definitions.h"
#include "drawing.h"
#include "eeprom.h"

#define VERBOSE_DEUG_LOGGING
#include "logging.h"

// BLOCK_XFER_BUFFER_SIZE is about 4k bytes
// one row of 800 pixels is 100 bytes so we can do 40 rows per pass
// requiring 12 passes / plane

#define BYTES_PER_PART  4000
#define LINES_PER_PART  40
#define TOTAL_PARTS     12

extern uint8_t blockXferBuffer[BLOCK_XFER_BUFFER_SIZE];
bool g2BitsPerPixel;
bool gBlackPass;
bool gDrawFromFlash;

uint16_t gPartY;
uint16_t gDrawY;
uint32_t gEEpromAdr;


#ifdef EPD_CODE

// Helper function to write to a pin
void Epd_DigitalWrite(int pin, int value) {
    GPIO_write(pin, value);
}

// Helper function to read a pin
int Epd_DigitalRead(int pin) {
    return GPIO_read(pin);
}

// Delay function in milliseconds
void Epd_DelayMs(unsigned int delaytime) {
    usleep(delaytime * 1000); // usleep accepts microseconds, so multiply by 1000
}

// Function to send 9 bits (DC bit + 8 data bits)
void write9bits(uint16_t data) {
   uint16_t Mask = 0x100;

   while(Mask != 0) {
      GPIO_write(CONFIG_GPIO_EPD_SDI_CONST, (data & Mask) ? 1 : 0);
      // Write each bit to the SDI pin
      // Pulse the clock (SCLK)
      GPIO_write(CONFIG_GPIO_EPD_CLK_CONST, 1);   // Clock high
      GPIO_write(CONFIG_GPIO_EPD_CLK_CONST, 0);   // Clock low
      Mask >>= 1;
   }
}

// Function to send a command
void Epd_SendCommand(uint8_t cmd) {
    GPIO_write(CONFIG_GPIO_EPD_CS_CONST, 0);  // Pull chip select low
    write9bits((0 << 8) | cmd);  // 0 for command, then the actual command byte
    GPIO_write(CONFIG_GPIO_EPD_CS_CONST, 1);  // Pull chip select high
}

// Function to send data
void Epd_SendData(uint8_t data) {
    GPIO_write(CONFIG_GPIO_EPD_CS_CONST, 0);  // Pull chip select low
    write9bits((1 << 8) | data);  // 1 for data, then the actual data byte
    GPIO_write(CONFIG_GPIO_EPD_CS_CONST, 1);  // Pull chip select high
}

// Function to reset the display
void Epd_Reset(void) {
    Epd_DigitalWrite(RST_PIN, 1);
    Epd_DelayMs(200);
    Epd_DigitalWrite(RST_PIN, 0);  // Module reset
    Epd_DelayMs(2);
    Epd_DigitalWrite(RST_PIN, 1);
    Epd_DelayMs(200);
}

// Function to wait until the display is idle
void Epd_WaitUntilIdle(void) {
    uint8_t busy;
    do {
        Epd_SendCommand(0x71); // Check busy status
        busy = Epd_DigitalRead(BUSY_PIN);
        busy = !(busy & 0x01);  // Busy pin is active low
    } while (busy);
    Epd_DelayMs(100);
}

// Initialize the e-paper display
// UC8179C like controller
void Epd_Init(void) {
    // Power on the display
    Epd_DigitalWrite(PWR_PIN, 0);

    // Reset the display
    Epd_Reset();

    // POWER SETTING command
    Epd_SendCommand(0x01);
    Epd_SendData(0x07);
    Epd_SendData(0x07); // Epd_SendData(0x17);
    Epd_SendData(0x3f);
    Epd_SendData(0x3f);

    // Booster soft start
    Epd_SendCommand(0x06);
    Epd_SendData(0x17);
    Epd_SendData(0x17);
    Epd_SendData(0x28); // Epd_SendData(0x27);
    Epd_SendData(0x17);

    // POWER ON
    Epd_SendCommand(0x04);
    Epd_DelayMs(100);
    Epd_WaitUntilIdle();

    // PANEL SETTING
    Epd_SendCommand(0x00);
    Epd_SendData(0x0F);

    // Set resolution (800x480)
    Epd_SendCommand(0x61);
    Epd_SendData(0x03);
    Epd_SendData(0x20);
    Epd_SendData(0x01);
    Epd_SendData(0xE0);

    // Dual SPI Mode
    Epd_SendCommand(0x15);
    Epd_SendData(0x00);

    // TCON setting
    Epd_SendCommand(0x60);
    Epd_SendData(0x22);

    // Vcom and data
    Epd_SendCommand(0x50);
    Epd_SendData(0x11);
    Epd_SendData(0x07);
}

// Enter deep sleep mode
void Epd_Sleep(void) {
    Epd_SendCommand(0x02);  // Enter deep sleep
    Epd_WaitUntilIdle();
    Epd_SendCommand(0x07);  // Power off
    Epd_SendData(0xa5);     // Sleep check code
}

void Epd_TurnOnDisplay(void) {
    Epd_SendCommand(0x12);
    Epd_DelayMs(100);
    Epd_WaitUntilIdle();
}

void Epd_ClearBlack(void)
{
    uint32_t Width =(EPD_WIDTH % 8 == 0)?(EPD_WIDTH / 8 ):(EPD_WIDTH / 8 + 1);
    uint32_t Height = EPD_HEIGHT;

    uint32_t i;
    Epd_SendCommand(0x10);
    for(i=0; i<Width*Height; i++) {
        Epd_SendData(0x00);

    }
    Epd_SendCommand(0x13);
    for(i=0; i<Width*Height; i++)	{
        Epd_SendData(0x00);

    }
    Epd_TurnOnDisplay();
}

void Epd_SendBlack(void)
{
    uint32_t Width =(EPD_WIDTH % 8 == 0)?(EPD_WIDTH / 8 ):(EPD_WIDTH / 8 + 1);
    uint32_t Height = EPD_HEIGHT;

    uint32_t i;
    for(i=0; i<Width*Height; i++) {
        Epd_SendData(0x00);

    }
}

void Epd_DrawPattern(void)
{
    uint32_t Width =(EPD_WIDTH % 8 == 0)?(EPD_WIDTH / 8 ):(EPD_WIDTH / 8 + 1);
    uint32_t Height = EPD_HEIGHT;

    uint32_t i;
    Epd_SendCommand(0x10);
    for(i=0; i<Width*Height; i++) {
        Epd_SendData(i % 0x100);
    }
    Epd_SendCommand(0x13);
    for(i=0; i<Width*Height; i++)	{
        Epd_SendData(0xFF - (i % 0x100));
    }
    Epd_TurnOnDisplay();
}

void DoPass()
{
   uint8_t Part;
   uint16_t i;
   uint8_t Mask = 0x80;
   uint8_t Value = 0;

   gPartY = 0;
   gDrawY = 0;

   VLOG("gEEpromAdr 0x%x\n",gEEpromAdr);

   for(Part = 0; Part < TOTAL_PARTS; Part++) {
      if(gDrawFromFlash) {
      // Read 40 lines of pixels
         if(!gBlackPass && !g2BitsPerPixel) {
         // Dummy pass
            memset(blockXferBuffer,0,BYTES_PER_PART);
         }
         else {
            eepromRead(gEEpromAdr,blockXferBuffer,BYTES_PER_PART);
            // VDUMP_HEX(blockXferBuffer,BYTES_PER_PART);
            gEEpromAdr += BYTES_PER_PART;
         }
         if(gBlackPass) {
            for(int i = 0; i < BYTES_PER_PART; i++) {
               Epd_SendData(~blockXferBuffer[i]);
            }
         }
         else {
            for(int i = 0; i < BYTES_PER_PART; i++) {
               Epd_SendData(blockXferBuffer[i]);
            }
         }
      }
#if 0
      else {
         xMemSet(blockXferBuffer,0,BYTES_PER_PART);
         for(i = 0; i < LINES_PER_PART; i++) {
            gDrawingFunct();
            gDrawY++;
         }
      }
#endif
      gPartY += LINES_PER_PART;
   }
}

void drawWithSleep(void)
{
    Epd_SendCommand(0x12);
    Epd_DelayMs(100);
    Epd_WaitUntilIdle();
}

void drawImageAtAddress(uint32_t addr, uint8_t lut)
{
   struct EepromImageHeader Hdr;

   gEEpromAdr = addr;
   LOG("Adr 0x%x lut %d\n",addr,lut);
   do {
      if(eepromRead(gEEpromAdr,&Hdr,sizeof(Hdr))) {
         break;
      }
      gEEpromAdr += sizeof(Hdr);

      if(Hdr.validMarker != EEPROM_IMG_VALID) {
         ELOG("Hdr is not valid\n");
         DUMP_HEX(&Hdr,sizeof(Hdr));
         break;
      }

      if(Hdr.dataType == DATATYPE_IMG_RAW_1BPP) {
         VLOG("DATATYPE_IMG_RAW_1BPP\n");
         g2BitsPerPixel = false;
      }
      else if(Hdr.dataType == DATATYPE_IMG_RAW_2BPP) {
         VLOG("DATATYPE_IMG_RAW_2BPP\n");
         g2BitsPerPixel = true;
      }
      else {
         ELOG("dataType 0x%x not supported\n",Hdr.dataType);
         break;
      }
      VLOG("size %u\n",Hdr.size);

      LOG("Drawing starting @ %u...\n",clock_time());
      Epd_Init();
      gDrawFromFlash = true;
      gBlackPass = true;
      Epd_SendCommand(0x10);
      DoPass();
      gBlackPass = false;
      Epd_SendCommand(0x13);
      DoPass();
      eepromPowerDown();
      drawWithSleep();
      Epd_Sleep();
      LOG("Drawing finished @ %u\n",clock_time());
   } while(false);
}

#endif   // EPD_CODE

#ifdef EPD_TEST
void Epd_Draw_Image(void)
{
    uint32_t Width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8) : (EPD_WIDTH / 8 + 1);  // Width in bytes, rounding up to handle partial bytes
    uint32_t Height = EPD_HEIGHT;
    uint32_t imgWidthInBytes = (IMG_WIDTH % 8 == 0) ? (IMG_WIDTH / 8) : (IMG_WIDTH / 8 + 1);  // Width of the image in bytes

    // Calculate the horizontal and vertical offsets to center the image
    uint32_t horizontal_offset = (EPD_WIDTH - IMG_WIDTH) / 2;
    uint32_t vertical_offset = (EPD_HEIGHT - IMG_HEIGHT) / 2;

    uint32_t i, j;
    uint8_t output;

    for (j = 0; j < Height; j++) {
        for (i = 0; i < Width; i++) {
            if (i >= horizontal_offset / 8 && i < (horizontal_offset + IMG_WIDTH) / 8 && j >= vertical_offset && j < (vertical_offset + IMG_HEIGHT)) {
                // If within the bounds of the image data, draw from img_data
                uint32_t imgIndex = (j - vertical_offset) * imgWidthInBytes + (i - horizontal_offset / 8);
                output = (imgIndex < IMG_DATA_SIZE) ? img_data[imgIndex] : 0xFF;  // Ensure we don't go out of bounds
            } else {
                // Else, fill with white (0xFF), which corresponds to 0b1 bits for each byte
                output = 0xFF;
            }
            Epd_SendData(output);
        }
    }
}

void Epd_Draw(void)
{
   Epd_SendCommand(0x10);
#ifdef EPD_TEST_BACKGROUND_WHITE
   Epd_Draw_Image();
   Epd_SendCommand(0x13);
   Epd_SendBlack();
#else
   Epd_SendBlack();
   Epd_SendCommand(0x13);
   Epd_Draw_Image();
#endif
    Epd_TurnOnDisplay();
}
#endif

