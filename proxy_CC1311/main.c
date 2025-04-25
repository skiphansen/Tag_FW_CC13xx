/*
 * Copyright (c) 2017-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/NVS.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti/drivers/gpio/GPIOCC26XX.h"
#include "ti_drivers_config.h"
#include "board.h"
#include "eeprom.h"
#include "CobsFraming.h"
#include "proxy_msgs.h"
#include "logging.h"

#define FW_VERSION      1
#define MAX_FRAME_IO_LEN      120
#define CMD_RESP  0x80

NVS_Handle gNvs;
extern UART2_Handle gDebugUart;
int gRxMsgLen;
// We need 3 extra bytes because the CRC is received into the buffer
// before we get the end of frame.  Additionally an 0 is appended to the
// received data for convenience
uint8_t gRxBuf[MAX_FRAME_IO_LEN+3];
uint8_t gTxBuf[MAX_FRAME_IO_LEN+3];

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



bool eepromInit()
{
   NVS_Params nvsParams;
   bool bRet = false;   // Assume the best

   if(gNvs == NULL) {
      NVS_Params_init(&nvsParams);

      gNvs = NVS_open(CONFIG_NVS_FLASH, &nvsParams);
      if(gNvs == NULL) {
         LOG("NVS_open() failed\n");
         bRet = true;
      }
   }
   return bRet;
}

uint32_t eepromGetSize(void)
{
   return EEPROM_SIZE;
}

bool eepromRead(uint32_t addr,void *pDst,uint32_t len)
{
   int_fast16_t Err;
   bool bRet = true; // assume the worse

   LOG("read 0x%x Len: %d\n", addr, len);
   do {
      if((addr + len) > EEPROM_SIZE) {
         ELOG("Invalid addr 0x%x\n",addr);
         break;
      }
      if(gNvs == NULL && eepromInit()) {
         break;
      }
      Err = NVS_read(gNvs,addr,pDst,len);
      if(Err != NVS_STATUS_SUCCESS) {
         LOG("NVS_read failed %d\n",Err);
         break;
      }
      LOG("read 0x%x Len: %d\n", addr, len);
      DUMP_HEX(pDst,len);
      bRet = false;
   } while(false);

   return bRet;
}

bool eepromWrite(uint32_t addr,void *pSrc,uint32_t len)
{
   int_fast16_t Err;
   bool bRet = true; // assume the worse

   LOG("write 0x%x Len: %d\n", addr, len);
   do {
      if((addr + len) > EEPROM_SIZE) {
         ELOG("Invalid addr 0x%x\n",addr);
         break;
      }
      if(gNvs == NULL && eepromInit()) {
         break;
      }
      Err = NVS_write(gNvs,addr,pSrc,len,0);
      if(Err != NVS_STATUS_SUCCESS) {
         LOG("NVS_write failed %d\n",Err);
         break;
      }
      bRet = false;
   } while(false);

   return bRet;
}

bool eepromErase(uint32_t addr,uint32_t len)
{
   int_fast16_t Err;
   int NumSectors = len / EEPROM_ERZ_SECTOR_SZ;
   bool bRet = true; // assume the worse

   LOG("erase 0x%x Len: %d\n",addr,len);
   do {
      if((addr % EEPROM_ERZ_SECTOR_SZ) != 0 ||
          addr > (EEPROM_SIZE - EEPROM_ERZ_SECTOR_SZ)) 
      {
         ELOG("Invalid addr 0x%x\n",addr);
         break;
      }
      if(gNvs == NULL && eepromInit()) {
         break;
      }

      if(NumSectors % EEPROM_ERZ_SECTOR_SZ != 0) {
         NumSectors++;
      }
      LOG("Erasing %d sectors\n",NumSectors);
      Err = NVS_erase(gNvs,addr,NumSectors * EEPROM_ERZ_SECTOR_SZ);
      if(Err != NVS_STATUS_SUCCESS) {
         LOG("NVS_erase failed %d\n",Err);
         break;
      }
      bRet = false;
   } while(false);

   return bRet;
}

bool eepromPowerDown()
{
   if(gNvs != NULL) {
      NVS_close(gNvs);
      gNvs = NULL;
   }

   return false;
}

bool eepromGetSFDP(void *pDst,uint32_t len)
{
   return true;
}

bool eepromGetID(void *pDst)
{
   return true;
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
            gRxBuf[1] = CMD_ERR_BUF_OVFL;
            break;
         }
         eepromRead(uCast1.Adr32,&gTxBuf[2],uCast0.IntValue);
         MsgLen += uCast0.IntValue;
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
         if(eepromGetSFDP(&gTxBuf[2],MAX_FRAME_IO_LEN - 2)) {
            ELOG("eepromGetSFDP failed\n");
            gTxBuf[1] = CMD_ERR_FAILED;
         }
         else {
            ALOG("Read:\n");
            DumpHex(&gTxBuf[2],MAX_FRAME_IO_LEN - 2);
            MsgLen = MAX_FRAME_IO_LEN;
         }
         break;

#if 0
      case CMD_EEPROM_WR:
      // uCast0 = Adr
         uCast0.Bytes[2] = gRxBuf[3];
#if 0
         LOG("write %d @ 0x%lx 0x%x 0x%x 0x%x\n",gRxMsgLen-4,uCast0.Adr32,
             gRxBuf[4],gRxBuf[5],gRxBuf[6]);
#endif
         u1setEepromMode();
         if(!eepromWrite(uCast0.Adr32,(void __xdata *) &gRxBuf[4],gRxMsgLen-4)) {
            gRxBuf[1] = CMD_ERR_FAILED;
         }
         u1setUartMode();
      // purge the receiver of any data received in SPI mode
         gUart1RxRd = gUart1RxWr;
         break;


      case CMD_BOARD_TYPE:
         MsgLen += strlen(gBuildType);
         memcpy(&gRxBuf[2],gBuildType,MsgLen-2);
         break;


      case CMD_RESET:
         if(MARCSTATE != MARC_STATE_IDLE) {
         // The watchdog won't timeout while transmitting !!!
            IdleMode(); 
         }
         wdtDeviceReset();
         break;

      case CMD_EEPROM_ERASE:
         LOG("Erase flash");
         if(gRxMsgLen == 1) {
         // Just command byte, erase entire chip
            u1setEepromMode();
            if(!eepromErase(0,32)) {
               gRxBuf[1] = CMD_ERR_FAILED;
            }
         }
         else if(gRxMsgLen == 6) {
         // Adr, # sectors
            LOG(", %d sectors from 0x%0lx",gRxBuf[5],uCast0.Uint32Value);
            u1setEepromMode();
            if(!eepromErase(uCast0.Uint32Value,gRxBuf[5])) {
               gRxBuf[1] = CMD_ERR_FAILED;
            }
         }
         else {
            LOG(", invalid arg");
            gRxBuf[1] = CMD_ERR_INVALID_ARG;
         }
         u1setUartMode();
         LOG("\n");
         break;

#endif
      default:
         LOG("Unknown command 0x%x ignored\n",gRxBuf[0]);
         gTxBuf[1] = CMD_ERR_INVALID_ARG;
         break;
   }

   if(MsgLen != 0) {
   // Send reply in gTxBuf
      SerialFrameIO_SendMsg(gTxBuf,MsgLen);
   }
}

void SerialFrameIO_SendByte(uint8_t TxByte)
{
   size_t Written;
   UART2_write(gDebugUart,&TxByte,1,&Written);
}

void Proxy()
{
   int BytesRx = 0;
   int MsgsRx = 0;

   SerialFrameIO_Init(gRxBuf,sizeof(gRxBuf));

   while(true) {
      uint8_t Byte;
      size_t BytesRead;
      int_fast16_t Err;
      while(true) {
         Err = UART2_read(gDebugUart,&Byte,1,&BytesRead);
         if(Err != UART2_STATUS_SUCCESS) {
            ELOG("UART2_read failed %d\n",Err);
            break;
         }
         if(BytesRead != 0) {
            break;
         }
      }
      if(Err != UART2_STATUS_SUCCESS) {
         break;
      }
      BytesRx++;
      gRxMsgLen = SerialFrameIO_ParseByte(Byte);
      if(gRxMsgLen == 0) {
      // Byte was not consumed, just echo it
         UART2_write(gDebugUart,&Byte,1,NULL);
      }
      else if(gRxMsgLen > 0) {
         MsgsRx++;
         HandleMsg();
      }
   }
}

void *mainThread(void *arg0)
{
   InitLogging();
   ALOG("CC1311 proxy Ver %d compiled " __DATE__ " " __TIME__ "\n",FW_VERSION);

   NVS_init();
   eepromInit();

   eepromRead(0,gTxBuf,4);
   Proxy();
   return(NULL);
}

