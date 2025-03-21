#ifndef _MAIN_H_
#define _MAIN_H_

//#define EPD_TEST
#define EPD_TEST_BACKGROUND_WHITE

// #define EPD_CODE
// #define NVS_DUMP
// #define NVS_TEST
// #define NVS_TEST_READBACK_ONLY
#define SERIAL_LOG

#ifdef SERIAL_LOG
void InitLogging(void);
int LogPrintf(char *fmt, ...);
#define LOG(format, ... ) LogPrintf(format,## __VA_ARGS__)
#else
#define InitLogging()
#define LOG(format, ... )
#endif


#endif   // _MAIN_H_

