#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <ti/drivers/UART.h>
#include "Board.h"
#include "logging.h"

#ifdef SERIAL_LOG
#if defined(DeviceFamily_CC13X0)
UART_Handle gDebugUart;
#endif

void InitLogging()
{
#if defined(DeviceFamily_CC13X0)
   UART_Params uartParams;
#endif

   UART_init();
   /* Create a UART with data processing off. */
   UART_Params_init(&uartParams);
   uartParams.writeDataMode = UART_DATA_BINARY;
   uartParams.readDataMode = UART_DATA_BINARY;
   uartParams.readReturnMode = UART_RETURN_FULL;
   uartParams.readEcho = UART_ECHO_OFF;
   uartParams.baudRate = 921600;

#if defined(DeviceFamily_CC13X0)
   gDebugUart = UART_open(Board_UART0, &uartParams);
#endif
   if(gDebugUart == NULL) {
       /* UART_open() failed */
       while (1);
   }
}

int LogPrintf(char *fmt, ...)
{
    va_list args;
    va_start(args,fmt);
    char  Temp[120];

    int Len = vsnprintf(Temp,sizeof(Temp),fmt,args);
    UART_write(gDebugUart,Temp,Len);

    return Len;
}

#if 0
void DumpHex(void *AdrIn,int Len)
{
   unsigned char *Adr = (unsigned char *) AdrIn;
   int i = 0;
   int j;

   while(i < Len) {
      for(j = 0; j < 16; j++) {
         if((i + j) == Len) {
            break;
         }
         LOG_RAW("%02x ",Adr[i+j]);
      }

      LOG_RAW(" ");
      for(j = 0; j < 16; j++) {
         if((i + j) == Len) {
            break;
         }
         if(isprint(Adr[i+j])) {
            LOG_RAW("%c",Adr[i+j]);
         }
         else {
            LOG_RAW(".");
         }
      }
      i += 16;
      LOG_RAW("\n");
   }
}
#else
// Source code version
void DumpHex(void *AdrIn,int Len)
{
   unsigned char *Adr = (unsigned char *) AdrIn;
   int i = 0;
   int j;

   while(i < Len) {
      for(j = 0; j < 16; j++) {
         if((i + j) == Len) {
            break;
         }
         LOG_RAW("0x%02x,",Adr[i+j]);
      }
      i += 16;
      LOG_RAW("\n");
   }
}
#endif


#endif   // SERIAL_LOG

