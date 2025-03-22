#ifndef _MAIN_H_
#define _MAIN_H_

#include <ti/drivers/NVS.h>

//#define EPD_TEST
#define EPD_TEST_BACKGROUND_WHITE

// #define EPD_CODE
// #define NVS_DUMP
// #define NVS_TEST
// #define NVS_TEST_READBACK_ONLY
#define WRITE_EPD_IMAGE

#define SERIAL_LOG

#ifdef SERIAL_LOG
void InitLogging(void);
int LogPrintf(char *fmt, ...);
void DumpHex(void *AdrIn,int Len);
#define LOG(format, ... ) LogPrintf(format,## __VA_ARGS__)
#define LOG_RAW(format, ... ) LogPrintf(format,## __VA_ARGS__)
#define DUMP_HEX(x,y) DumpHex(x,y)
#else
#define InitLogging()
#define LOG(format, ... )
#define LOG_RAW(format, ... )
#define DUMP_HEX(x,y) DumpHex(x,y)
#endif

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


#endif   // _MAIN_H_

