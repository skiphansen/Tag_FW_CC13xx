#include <stdio.h>
#include <string.h>
#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"
#include "main.h"

UART2_Handle gDebugUart;

#ifdef SERIAL_LOG
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

int LogPrintf(char *fmt, ...)
{
    va_list args;
    va_start(args,fmt);
    char  Temp[120];

    int Len = vsnprintf(Temp,sizeof(Temp),fmt,args);
    UART2_write(gDebugUart,Temp,Len,NULL);

    return Len;
}
#endif   // SERIAL_LOG

