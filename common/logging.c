#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "logging.h"

#if defined(DeviceFamily_CC13X0)
#include <ti/drivers/UART.h>
#include "Board.h"
UART_Handle gDebugUart;
#endif

#if defined(DeviceFamily_CC13X1)
#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"
UART2_Handle gDebugUart;
#endif


#if defined(DeviceFamily_CC13X0)
void InitLogging()
{
   UART_Params uartParams;

   UART_init();
   /* Create a UART with data processing off. */
   UART_Params_init(&uartParams);
   uartParams.writeDataMode = UART_DATA_BINARY;
   uartParams.readDataMode = UART_DATA_BINARY;
   uartParams.readReturnMode = UART_RETURN_FULL;
   uartParams.readEcho = UART_ECHO_OFF;
   uartParams.baudRate = 921600;

   gDebugUart = UART_open(Board_UART0, &uartParams);
   if(gDebugUart == NULL) {
       /* UART_open() failed */
       while (1);
   }
}
#endif

#if defined(DeviceFamily_CC13X1)
void InitLogging()
{
    UART2_Params uartParams;
    UART2_Handle uart;

    UART2_Params_init(&uartParams);
    uartParams.baudRate = 921600;
    gDebugUart = UART2_open(DEBUG_UART,&uartParams);

    if(gDebugUart == NULL) {
    // UART2_open() failed
        while(1) {
        }
    }
}
#endif

#if SERIAL_LOG == 1
int LogPrintf(char *fmt, ...)
{
    va_list args;
    va_start(args,fmt);
    char  Temp[120];

    int Len = vsnprintf(Temp,sizeof(Temp),fmt,args);
#if defined(DeviceFamily_CC13X0)
    UART_write(gDebugUart,Temp,Len);
#endif

#if defined(DeviceFamily_CC13X1)
    UART2_write(gDebugUart,Temp,Len,NULL);
#endif

    return Len;
}
#endif   // SERIAL_LOG

#if 1
// Debugging version
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
// version to allow hex dumps to be copied to source files easily
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


