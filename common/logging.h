#ifndef _LOGGING_H_
#define _LOGGING_H_

#ifdef SERIAL_LOG
   void InitLogging(void);
   int LogPrintf(char *fmt, ...);
   #define _LOG(format, ...) LogPrintf(format,## __VA_ARGS__)
#else
   #define InitLogging()
   #define _LOG(format, ...)
#endif   // SERIAL_LOG

// LOGA always logs
#define LOGA(format, ... ) _LOG(format,## __VA_ARGS__)

// LOGE always logs
#define LOGE(format, ...) _LOG("%s#%d: " format,__FUNCTION__,__LINE__,## __VA_ARGS__)

#ifdef DEBUG_LOGGING
   void DumpHex(void *AdrIn,int Len);
   #define LOG(format, ... ) _LOG("%u %s: " format,clock_time(),__FUNCTION__,## __VA_ARGS__)
   #define LOG_RAW(format, ... ) _LOG(format,## __VA_ARGS__)
   #define DUMP_HEX(x,y) DumpHex(x,y)
#else
   #define LOG(format, ... )
   #define LOG_RAW(format, ... )
   #define DUMP_HEX(x,y)
#endif   // DEBUG_LOGGING
#endif   // _LOGGING_H_

