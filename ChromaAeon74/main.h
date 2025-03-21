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


#endif   // _MAIN_H_

