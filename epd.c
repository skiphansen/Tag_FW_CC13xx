#include "epd.h"
#include <unistd.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"

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

// Helper function to send a single byte over SPI (bit-banging)
void spi_send_byte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        // Write each bit to the SDI pin
        GPIO_write(CONFIG_GPIO_EPD_SDI_CONST, (data & (1 << i)) ? 1 : 0);
        // Pulse the clock (SCLK)
        GPIO_write(CONFIG_GPIO_EPD_CLK_CONST, 1);   // Clock high
        GPIO_write(CONFIG_GPIO_EPD_CLK_CONST, 0);   // Clock low
    }
}

// Function to send 9 bits (DC bit + 8 data bits)
void write9bits(uint16_t data) {
    // Send the upper 8 bits
    spi_send_byte((uint8_t)(data >> 1));  // Mask the lower bit
    // Send the lower bit and a padding zero bit
    spi_send_byte((uint8_t)((data & 1) << 7));  // Shift the lower bit to MSB and send
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
    Epd_DelayMs(200);
}

// Initialize the e-paper display
void Epd_Init(void) {
    // Power on the display
    Epd_DigitalWrite(PWR_PIN, 1);
    Epd_Reset();

    // POWER SETTING command
    Epd_SendCommand(0x01);
    Epd_SendData(0x07);
    Epd_SendData(0x07);  // VGH=20V, VGL=-20V
    Epd_SendData(0x3f);  // VDH=15V
    Epd_SendData(0x3f);  // VDL=-15V

    // POWER ON
    Epd_SendCommand(0x04);
    Epd_DelayMs(100);
    Epd_WaitUntilIdle();

    // PANEL SETTING
    Epd_SendCommand(0x00);
    Epd_SendData(0x0F);  // KW-3f, KWR-2F, BWROTP 0f, BWOTP 1f

    // Set resolution (800x480)
    Epd_SendCommand(0x61);
    Epd_SendData(0x03);
    Epd_SendData(0x20);
    Epd_SendData(0x01);
    Epd_SendData(0xE0);

    // Other settings
    Epd_SendCommand(0x15);
    Epd_SendData(0x00);

    Epd_SendCommand(0x50);
    Epd_SendData(0x11);
    Epd_SendData(0x07);

    Epd_SendCommand(0x60);
    Epd_SendData(0x22);

    Epd_SendCommand(0x65);
    Epd_SendData(0x00);
    Epd_SendData(0x00);  // 800x480 resolution
    Epd_SendData(0x00);
    Epd_SendData(0x00);
}

// Enter deep sleep mode
void Epd_Sleep(void) {
    Epd_SendCommand(0x02);  // Enter deep sleep
    Epd_WaitUntilIdle();
    Epd_SendCommand(0x07);  // Power off
    Epd_SendData(0xa5);     // Sleep check code
}

// Display a part of the image (specified by xStart, yStart, width, height)
void Epd_DisplayPart(uint16_t xStart, uint16_t yStart, uint16_t width, uint16_t height) {
    uint16_t x, y;
    for (y = yStart; y < yStart + height; y++) {
        Epd_SendCommand(0x24); // Memory write command
        for (x = xStart; x < xStart + width; x++) {
            Epd_SendData(0xFF); // Send one byte of image data at a time
        }
    }
}
