/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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

/*
 *  ======== rfEasyLinkEchoTx_nortos.c ========
 */
 /* Standard C Libraries */
#include <stdlib.h>

/* TI-RTOS Header files */
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/devices/DeviceFamily.h>

#include "oepl-definitions.h"
#include "oepl-proto.h"

#include "logging.h"

/* Driverlib APIs */
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "smartrf_settings/smartrf_settings.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/* Undefine to not use async mode */
#define RFEASYLINKECHO_ASYNC

#define RFEASYLINKECHO_PAYLOAD_LENGTH     30

// #define SPI_TEST
#define CHROMA_PROXY


static uint8_t CreatePingPacket(void);


#ifdef RFEASYLINKECHO_ASYNC
/* GPTimer handle and timeout value */
GPTimerCC26XX_Handle hTimer;
GPTimerCC26XX_Value rxTimeoutVal;

/* GP Timer Callback */
void rxTimeoutCb(GPTimerCC26XX_Handle handle,
                 GPTimerCC26XX_IntMask interruptMask);

static volatile bool rxDoneFlag;
static volatile bool rxTimeoutFlag;
#endif

static volatile bool bEchoDoneFlag;
static volatile int gRxLen;
static volatile uint8_t *gRxData;

EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};

#ifdef RFEASYLINKECHO_ASYNC
void echoTxDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
    }
    else {
       ELOG("status %d\n",status);
       while(true);
    }

    bEchoDoneFlag = true;
}

void echoRxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    uint32_t currTimerVal;

    if(status == EasyLink_Status_Success) {
//       LOG("Received %d byte packet:\n",rxPacket->len);
//       DUMP_HEX(rxPacket->payload,rxPacket->len);
       gRxLen = rxPacket->len;
       gRxData = rxPacket->payload;
        /*
         * Stop the Receiver timeout timer, find the current free-running
         * counter value and add it to the existing interval load value
         */
#if 0
        GPTimerCC26XX_stop(hTimer);
        currTimerVal = GPTimerCC26XX_getValue(hTimer);
        GPTimerCC26XX_setLoadValue(hTimer, rxTimeoutVal + currTimerVal);
#endif
    }
    else if (status == EasyLink_Status_Aborted) {
//       LOG("Receive aborted\n");
    }
    else
    {
//       LOG("status %d\n",status);
        /*
         * Stop the Receiver timeout timer, find the current free-running
         * counter value and add it to the existing interval load value
         */
#if 0
        GPTimerCC26XX_stop(hTimer);
        currTimerVal = GPTimerCC26XX_getValue(hTimer);
        GPTimerCC26XX_setLoadValue(hTimer, rxTimeoutVal + currTimerVal);
#endif
    }

    bEchoDoneFlag = true;
}
#endif //RFEASYLINKECHO_ASYNC

uint8_t gChannel = 203;
// rfc_CMD_FS_t.frequency, rfc_CMD_FS_t.fractFreq values for 868Mhz band
const uint16_t g915SynthValues[6][2] = {
   {0x387,0x0000},   // Channel 200 / CHANNR 0:  902.999756 Mhz
   {0x38b,0x0700},   // Channel 201 / CHANNR 12: 907.027344 Mhz
   {0x38f,0x0E00},   // Channel 202 / CHANNR 24: 911.054932 Mhz
   {0x393,0x1534},   // Channel 203 / CHANNR 36: 915.082520 Mhz
   {0x397,0x1C34},   // Channel 204 / CHANNR 48: 919.110107 Mhz
   {0x39b,0x2334}   // Channel 205 / CHANNR 60: 923.137695 Mhz
};

#ifdef SPI_TEST
#include "eeprom.h"

uint8_t RxBuf[4096];
void SpiTest()
{
   uint32_t addr = 0;

   memset(RxBuf,0xaa,sizeof(RxBuf));
   do {
      if(eepromGetSFDP(RxBuf,128)) {
         LOG("eepromGetSFDP failed\n");
         break;
      }
      LOG("SFDP:\n");
      DUMP_HEX(RxBuf,128);
      if(eepromRead(0,RxBuf,sizeof(RxBuf))) {
         LOG("eepromRead failed\n");
         break;
      }
      while(addr < 128*1024) {
         LOG("0x%x:\n",addr);
         if(eepromRead(addr,RxBuf,sizeof(RxBuf))) {
            LOG("eepromRead failed\n");
            break;
         }
         DUMP_HEX(RxBuf,sizeof(RxBuf));
         addr += 4096;
      }

   } while(false);
   while(true);
}
#endif   // SPI_TEST

#ifdef CHROMA_PROXY
#include "CobsFraming.h"
#include "proxy_msgs.h"
#include <ti/drivers/UART.h>

extern UART_Handle gDebugUart;
int gRxMsgLen;
#define MAX_FRAME_IO_LEN      120
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

#define CMD_RESP  0x80
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
   UART_write(gDebugUart,&TxByte,1);
}

void Proxy()
{
   int BytesRx = 0;
   int MsgsRx = 0;

   SerialFrameIO_Init(gRxBuf,sizeof(gRxBuf));

   while(true) {
      uint8_t Byte;
      while(UART_read(gDebugUart,&Byte,1) == 0);
      BytesRx++;
      gRxMsgLen = SerialFrameIO_ParseByte(Byte);
      if(gRxMsgLen == 0) {
      // Byte was not consumed, just echo it
         UART_write(gDebugUart,&Byte,1);
      }
      else if(gRxMsgLen > 0) {
         MsgsRx++;
         HandleMsg();
      }
   }
}
#endif   // CHROMA_PROXY


void *mainThread(void *arg0)
{
    EasyLink_Status Err;
    int TxPings = 1;

    InitLogging();
    ALOG("rfEchoTx_CC1310 compiled " __DATE__ " " __TIME__ "\n");
#ifdef SPI_TEST
    SpiTest();
    while(true);
#endif

#ifdef CHROMA_PROXY
    Proxy();
#endif

#ifdef RFEASYLINKECHO_ASYNC
    /* Reset the timeout flag */
    rxTimeoutFlag = false;
    /* Set the echo flag to its default state */
    bEchoDoneFlag = false;

    /* Open the GPTimer driver */
    GPTimerCC26XX_Params params;
    GPTimerCC26XX_Params_init(&params);
    params.width          = GPT_CONFIG_32BIT;
    params.mode           = GPT_MODE_ONESHOT;
    params.direction      = GPTimerCC26XX_DIRECTION_UP;
    params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hTimer = GPTimerCC26XX_open(Board_GPTIMER0A, &params);
    if(hTimer == NULL)
    {
       ELOG("GPTimerCC26XX_open failed\n");
        while(1);
    }

    /* Set Timeout value to 1 second */
    rxTimeoutVal = (SysCtrlClockGet()*10UL)/10UL - 1UL;
    GPTimerCC26XX_setLoadValue(hTimer, rxTimeoutVal);


    /* Register the GPTimer interrupt */
    GPTimerCC26XX_registerInterrupt(hTimer, rxTimeoutCb, GPT_INT_TIMEOUT);
#else
    EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};
#endif //RFEASYLINKECHO_ASYNC

    RF_cmdPropRadioDivSetup.centerFreq = g915SynthValues[gChannel-200][0];
    RF_cmdFs.frequency = g915SynthValues[gChannel-200][0];
    RF_cmdFs.fractFreq = g915SynthValues[gChannel-200][1];

    // Initialize the EasyLink parameters to their default values
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

    /*
     * Initialize EasyLink with the settings found in easylink_config.h
     * Modify EASYLINK_PARAM_CONFIG in easylink_config.h to change the default
     * PHY
     */
    if(( Err = EasyLink_init(&easyLink_params)) != EasyLink_Status_Success) {
       ELOG("EasyLink_init failed\n",Err);
    }

    /*
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     */

    // Packet Originator
    while(1) {
       gRxLen = 0;

        txPacket.len = CreatePingPacket();
        txPacket.absTime = 0;

        LOG("Sending ping %d...",TxPings++);
        /* Set Echo flag to false, TX Cb should set it to true */
        bEchoDoneFlag = false;
        Err = EasyLink_transmitAsync(&txPacket, echoTxDoneCb);
        if(Err != EasyLink_Status_Success) {
           ELOG("\nEasyLink_transmitAsync failed %d\n",Err);
           break;
        }

        /* Wait for Tx to complete. A Successful TX will cause the echoTxDoneCb
         * to be called and the echoDoneSem to be released, so we must
         * consume the echoDoneSem
         */
        while(bEchoDoneFlag == false){};
        LOG_RAW("\n");

        /* Switch to Receiver */
        bEchoDoneFlag = false;
        EasyLink_receiveAsync(echoRxDoneCb, 0);

        /*
         * Start the Receiver timeout timer (500ms) before
         * EasyLink_receiveAsync enables the power policy
         */
        GPTimerCC26XX_start(hTimer);

        while(true) {
            bool previousHwiState = IntMasterDisable();
            /*
             * Tricky IntMasterDisable():
             * true  : Interrupts were already disabled when the function was
             *         called.
             * false : Interrupts were enabled and are now disabled.
             */
            IntMasterEnable();
            Power_idleFunc();
            IntMasterDisable();

            if(!previousHwiState)
            {
                IntMasterEnable();
            }

            /* Break if timeout flag is set */
            if(rxTimeoutFlag == true) {
            /* Reset the timeout flag */
               rxTimeoutFlag = false;
               if(gRxLen == 0) {
                  LOG("Timeout\n");
                   /* RX timed out, abort */
                   if((Err = EasyLink_abort()) == EasyLink_Status_Success)
                   {
                      LOG("Waiting for receive to abort ...");
                       /* Wait for the abort */
                       while(bEchoDoneFlag == false){};
                       LOG_RAW("\n");
                   }
                   else {
                      ELOG("EasyLink_abort failed %d\n",Err);
                   }
               }
               break;
            }

            if(gRxLen != 0) {
               LOG("Received %d byte packet:\n",gRxLen);
               DUMP_HEX(gRxData,gRxLen);
            }
        }
    }

    return NULL;
}

#ifdef RFEASYLINKECHO_ASYNC
/* GP Timer Callback Function */
void rxTimeoutCb(GPTimerCC26XX_Handle handle,
                 GPTimerCC26XX_IntMask interruptMask)
{
    /* Set the Timeout Flag */
    rxTimeoutFlag = true;
#if 0
    /*
     * Timer is automatically stopped in one-shot mode and needs to be reset by
     * loading the interval load value
     */
    GPTimerCC26XX_setLoadValue(hTimer, rxTimeoutVal);
#endif
}
#endif // RFEASYLINKECHO_ASYNC

uint8_t mSelfMac[8];
uint8_t gSeq;

// radio stuff
/* 
21 c8 19 47 44 ff ff 47 44 78 56 34 12 11 12 67  !..GD..GDxV4...g 
21 c8 
0c 
47 44 
ff ff 
47 44 
78 56 34 12 11 12 67 
 
21 c8 
08 
47 44 
ff ff 
47 44 
78 56 34 12 11 12 67 44 
00  
*/

static uint8_t CreatePingPacket()
{
    struct MacFrameBcast *txframe = (struct MacFrameBcast *)&txPacket.payload;
    memset(txframe, 0, sizeof(struct MacFrameBcast));

    mSelfMac[7] = 0x44;
    mSelfMac[6] = 0x67;

    mSelfMac[5] = 'S' - 'A';
    mSelfMac[4] = 'R' - 'A';

    mSelfMac[3] = 0x12;
    mSelfMac[2] = 0x34;
    mSelfMac[1] = 0x56;
    mSelfMac[0] = 0x78;

    memcpy(txframe->src, mSelfMac, 8);
    txframe->fcs.frameType = 1;
    txframe->fcs.ackReqd = 1;
    txframe->fcs.destAddrType = 2;
    txframe->fcs.srcAddrType = 3;
    txframe->seq = gSeq++;
    txframe->dstPan = PROTO_PAN_ID_SUBGHZ;
    txframe->dstAddr = 0xFFFF;
    txframe->srcPan = PROTO_PAN_ID_SUBGHZ;

    (&txframe->src[7])[1] = PKT_PING;

    return (uint8_t) sizeof(*txframe) + 3;   // NB: Tx len includes CRC
}

