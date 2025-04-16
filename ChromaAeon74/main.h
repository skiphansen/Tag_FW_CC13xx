#ifndef _MAIN_H_
#define _MAIN_H_

#include <ti/drivers/NVS.h>

//#define EPD_TEST
//#define EPD_TEST_BACKGROUND_WHITE

#define EPD_CODE
// #define NVS_DUMP
// #define NVS_TEST
// #define NVS_TEST_READBACK_ONLY
// #define WRITE_EPD_IMAGE
// #define SPI_TEST

#define DEBUG_MAX_SLEEP    5000
// version number (
// NB: the first version # that will be displayed by the AP is 2
#define FW_VERSION         0x0003

#define SERIAL_LOG
#define DEBUG_LOGGING

#if defined(EPD_TEST) || defined(WRITE_EPD_IMAGE)
#define EPD_TEST_IMAGE_DATA
#endif

#if defined(NVS_DUMP) || defined(NVS_TEST) || defined(WRITE_EPD_IMAGE)
#define ENABLE_NVS
#endif

NVS_Handle gNvs;

#define TEMP_BUF_SIZE   4096
extern uint8_t gTempBuf[TEMP_BUF_SIZE];

extern uint8_t mSelfMac[8];

// Test routines in test_code.c
#ifdef WRITE_EPD_IMAGE
void WriteEpdImage(void);
#else
#define WriteEpdImage()
#endif

#ifdef EPD_TEST
void EpdTest(void);
#else
#define EpdTest()
#endif

#ifdef NVS_DUMP
void NvrDump(void);
#else
#define NvrDump()
#endif

#ifdef NVS_TEST
void NvrTest(void);
#else
#define NvrTest()
#endif

#ifdef SPI_TEST
void SpiTest(void);
#else
#define SpiTest()
#endif

uint16_t get_battery_mv();
void wdt10s(void);
void wdt60s(void);


extern uint8_t wakeUpReason;
extern int8_t  temperature;
extern uint16_t  batteryVoltage;
extern bool  lowBattery;
extern uint8_t capabilities;


#endif   // _MAIN_H_

