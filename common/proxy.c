#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <ti/devices/DeviceFamily.h>
#include <ti/drivers/GPIO.h>

#include "board.h"
#include "eeprom.h"
#include "CobsFraming.h"
#include "proxy_msgs.h"
#include "logging.h"

#define COBS_BUF_LEN      120
#define CMD_RESP  0x80

typedef union {
   int IntValue;
   uint32_t Uint32Value;
   int32_t Int32Value;
   uint16_t Adr;
   uint32_t Adr32;
   uint8_t *pXdata; 
   uint8_t Bytes[4];
   uint8_t Uint8Value;
} CastUnion;

const char gBuildType[] = BOARD_NAME;
int gMaxMsgLen;

#if defined(DeviceFamily_CC13X0)
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#include <ti/drivers/UART.h>
#include "Board.h"
extern UART_Handle gDebugUart;

bool UartReadByte(uint8_t *Byte)
{
   while(UART_read(gDebugUart,Byte,1) == 0);
   return false;
}

void SerialFrameIO_SendByte(uint8_t TxByte)
{
   UART_write(gDebugUart,&TxByte,1);
}

#elif defined(DeviceFamily_CC13X1)
#include <ti/drivers/UART2.h>
#include <ti/devices//cc13x1_cc26x1/driverlib/sys_ctrl.h>
#include "ti_drivers_config.h"
extern UART2_Handle gDebugUart;
bool UartReadByte(uint8_t *Byte)
{
   bool bRet = false;   // Assume the best
   size_t BytesRead;
   int_fast16_t Err;

   Err = UART2_read(gDebugUart,Byte,1,&BytesRead);
   if(Err != UART2_STATUS_SUCCESS) {
      ELOG("UART2_read failed %d\n",Err);
      bRet = true;
   }
   return bRet;
}

void SerialFrameIO_SendByte(uint8_t TxByte)
{
   size_t Written;

   UART2_write(gDebugUart,&TxByte,1,&Written);
}
#else
#error "Unsupported deviceFamily"
#endif

int gRxMsgLen;
uint8_t gRxBuf[COBS_BUF_LEN];
uint8_t gTxBuf[COBS_BUF_LEN];

void write9bits(uint16_t data) {
   uint16_t Mask = 0x100;

   while(Mask != 0) {
      GPIO_write(CONFIG_GPIO_EPD_SDI, (data & Mask) ? 1 : 0);
      // Write each bit to the SDI pin
      // Pulse the clock (SCLK)
      GPIO_write(CONFIG_GPIO_EPD_CLK, 1);   // Clock high
      GPIO_write(CONFIG_GPIO_EPD_CLK, 0);   // Clock low
      Mask >>= 1;
   }
}

// <CMD_EPD> <Flags> [<CommandBytes> <Command> [<Data>] ...]
void EpdCmd()
{
   int MsgLen = 1;
   uint8_t Flags = gRxBuf[MsgLen++];

// LOG("Got EpdCmd, gRxMsgLen %d, flags 0x%x\n",gRxMsgLen,Flags);

   if(Flags & EPD_FLG_RESET) {
   // reset display (active low)
      GPIO_write(CONFIG_GPIO_EPD_RST,0);
   }
   else {
   // release reset
      GPIO_write(CONFIG_GPIO_EPD_RST,1);
   }

   if(Flags & EPD_FLG_ENABLE) {
   // turn off the eInk power
      GPIO_write(CONFIG_GPIO_EPD_PWR,1);
   }
   else {
   // turn on the eInk power
      GPIO_write(CONFIG_GPIO_EPD_PWR,0);
   }

   if(gRxMsgLen > 2) {
   // we have data
      uint8_t CmdBytes = gRxBuf[MsgLen++];
      if(CmdBytes > 0) {
         GPIO_write(CONFIG_GPIO_EPD_CS, 0);
         while(MsgLen < gRxMsgLen) {
            if(Flags & EPD_FLG_CMD) {
            // Send command byte
               Flags &= ~EPD_FLG_CMD;
               write9bits(gRxBuf[MsgLen++]);
            }
            else {
            // Send Data byte
               write9bits(0x100 | gRxBuf[MsgLen++]);
            }
         }
      // set nCS high
         GPIO_write(CONFIG_GPIO_EPD_CS,1);
      }
   }
}

// commands <CmdByte> <command data>
// respones <CmdByte | 0x80> <Rcode> <response data>
void HandleMsg()
{
   int MsgLen = 2;
   CastUnion uCast0;
   CastUnion uCast1;
   CastUnion *pResponse = (CastUnion *) &gTxBuf[2];

// Assume that the first two arguments are 16 bits
   uCast0.Bytes[0] = gRxBuf[1];
   uCast0.Bytes[1] = gRxBuf[2];
   uCast0.Bytes[2] = uCast0.Bytes[3] = 0;
   uCast1.Bytes[0] = gRxBuf[3];
   uCast1.Bytes[1] = gRxBuf[4];
   uCast1.Bytes[2] = uCast1.Bytes[3] = 0;

   LOG("Got %d byte message:\n",gRxMsgLen);
   DUMP_HEX(gRxBuf,gRxMsgLen);

   gTxBuf[0] = gRxBuf[0] | CMD_RESP;
   gTxBuf[1] = CMD_ERR_NONE;

   switch(gRxBuf[0] & ~CMD_RESP) {
      case CMD_PING:
         LOG("Got ping\n");
         break;

      case CMD_EEPROM_RD:
      // uCast1 = Adr
      // uCast0 = Len
         uCast1.Bytes[2] = gRxBuf[5];
         if(uCast0.IntValue > sizeof(gRxBuf) - 2) {
            gTxBuf[1] = CMD_ERR_BUF_OVFL;
            break;
         }
         if(eepromRead(uCast1.Adr32,&gTxBuf[2],uCast0.IntValue)) {
            gTxBuf[1] = CMD_ERR_FAILED;
         }
         else {
            MsgLen += uCast0.IntValue;
         }
         break;

      case CMD_EEPROM_LEN:
         pResponse->Uint32Value = eepromGetSize();
         MsgLen += sizeof(uCast1.Uint32Value);
         break;

      case CMD_COMM_BUF_LEN:
         pResponse->IntValue = sizeof(gRxBuf);
         MsgLen += sizeof(uCast1.IntValue);
         break;

      case CMD_EEPROM_ID:
         if(eepromGetID(&gTxBuf[2])) {
            LOG("eepromGetID failed\n");
            gTxBuf[1] = CMD_ERR_FAILED;
         }
         else {
            MsgLen += 2;
         }
         break;

      case CMD_READ_SFDP:
         if(eepromGetSFDP(&gTxBuf[2],gMaxMsgLen - 2)) {
            ELOG("eepromGetSFDP failed\n");
            gTxBuf[1] = CMD_ERR_FAILED;
         }
         else {
            VLOG("Read:\n");
            VDUMP_HEX(&gTxBuf[2],gMaxMsgLen - 2);
            MsgLen = gMaxMsgLen;
         }
         break;

      case CMD_EEPROM_PD:
         if(gRxMsgLen != 2 || gRxBuf[1] > 1) {
            gTxBuf[1] = CMD_ERR_INVALID_ARG;
         }
         else if(eepromPower(gRxBuf[1])) {
            ELOG("eepromPowerDown failed\n");
            gTxBuf[1] = CMD_ERR_FAILED;
         }
         break;

      case CMD_EEPROM_WR:
      // uCast0 = Adr
         uCast0.Bytes[2] = gRxBuf[3];
#if 0
         LOG("write %d @ 0x%lx 0x%x 0x%x 0x%x\n",gRxMsgLen-4,uCast0.Adr32,
             gRxBuf[4],gRxBuf[5],gRxBuf[6]);
#endif
         if(eepromWrite(uCast0.Adr32,&gRxBuf[4],gRxMsgLen-4)) {
            gTxBuf[1] = CMD_ERR_FAILED;
         }
         break;

      case CMD_BOARD_TYPE:
         MsgLen += strlen(gBuildType);
         memcpy(&gTxBuf[2],gBuildType,MsgLen-2);
         break;

      case CMD_RESET:
         while(true) {
            GPIO_write(CONFIG_GPIO_EPD_RST,0);
            GPIO_write(CONFIG_GPIO_EPD_RST,1);
         }
         SysCtrlSystemReset();
         break;

      case CMD_EEPROM_ERASE:
         LOG("Erase flash");
         if(gRxMsgLen == 1) {
         // Just command byte, erase entire chip
            if(eepromErase(0,EEPROM_SIZE)) {
               gRxBuf[1] = CMD_ERR_FAILED;
            }
         }
         else if(gRxMsgLen == 6) {
         // Adr, # sectors
            LOG(", %d sectors from 0x%0lx",gRxBuf[5],uCast0.Uint32Value);
            if(eepromErase(uCast0.Uint32Value,gRxBuf[5] * EEPROM_ERZ_SECTOR_SZ)) {
               gTxBuf[1] = CMD_ERR_FAILED;
            }
         }
         else {
            LOG(", invalid arg");
            gTxBuf[1] = CMD_ERR_INVALID_ARG;
         }
         LOG("\n");
         break;

      case CMD_EPD:
         if(gRxMsgLen > 1) {
            EpdCmd();
         }
         else {
            gTxBuf[1] = CMD_ERR_INVALID_ARG;
         }
         break;

      case CMD_PORT_RW:
      // CMD_PORT_RW <port> <mask> <value>
         if(uCast0.Bytes[0] == 1 && uCast0.Bytes[1] == 0) {
         // Reading "port 1"
            MsgLen = 3;
            pResponse->Bytes[0] = GPIO_read(CONFIG_GPIO_EPD_BUSY);
         }
         else {
            gTxBuf[1] = CMD_ERR_INVALID_ARG;
         }
         break;

      default:
         LOG("Unknown command 0x%x ignored\n",gRxBuf[0]);
         gTxBuf[1] = CMD_ERR_UNKNOWN_CMD;
         break;
   }

   if(MsgLen != 0) {
   // Send reply in gTxBuf
      SerialFrameIO_SendMsg(gTxBuf,MsgLen);
   }
}

void Proxy()
{
   int BytesRx = 0;
   int MsgsRx = 0;

   gMaxMsgLen = SerialFrameIO_Init(gRxBuf,sizeof(gRxBuf));

   while(true) {
      uint8_t Byte;
      if(UartReadByte(&Byte)) {
         break;
      }
      BytesRx++;
      gRxMsgLen = SerialFrameIO_ParseByte(Byte);
      if(gRxMsgLen > 0) {
         MsgsRx++;
         HandleMsg();
      }
   }
}

