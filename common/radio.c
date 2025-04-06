#include <stdlib.h>
#include <ti/display/Display.h>
#include <ti/drivers/rf/RF.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include "ti_drivers_config.h"
#include "RFQueue.h"
#include <ti_radio_config.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
#include <ti/drivers/dpl/ClockP.h>

#include "oepl-definitions.h"
#include "oepl-proto.h"
#include "cc13xx.h"
#include "radio.h"
#include "main.h"
#include "logging.h"

#define RX_TIMEOUT          (uint32_t)(1000000)
#define NUM_DATA_ENTRIES    2

// Proprietary Mode Transmit Command
#define RF_cmdPropTx RF_cmdPropTx_custom868_0

// Proprietary Mode Receive Command
#define RF_cmdPropRx RF_cmdPropRx_custom868_0

// Proprietary Mode Radio Setup Command for All Frequency Bands
#define RF_cmdPropRadioDivSetup RF_cmdPropRadioDivSetup_custom868_0

// Custom RF Setting:
// 
// RX Address Mode: No address check
// Frequency (MHz): 863.9899
// Deviation (kHz): 165.0
// Packet Length Config: Variable
// Max Packet Length: 255
// Preamble Count: 4 Bytes
// Preamble Mode: Send 0 as the first preamble bit
// RX Filter BW (kHz): 621.6
// Symbol Rate (kBaud): 250.000
// Sync Word: 0xD391D391
// Sync Word Length: 32 Bits
// TX Power (dBm): 10
// Whitening: CC1101/CC2500 compatible
#define RF_prop RF_prop_custom868_0

// Frequency Synthesizer Programming Command
#define RF_cmdFs RF_cmdFs_custom868_0

// rfc_CMD_FS_t.frequency, rfc_CMD_FS_t.fractFreq values for 868Mhz band
const uint16_t gSynthValues[] = {
   0x360,0x0000, // Channel 100: 863.999756 Mhz
   0x361,0x01CD, // Channel 101: 865.006653 Mhz
   0x362,0x0367, // Channel 102: 866.013550 Mhz
   0x363,0x0534, // Channel 103: 867.020447 Mhz
   0x364,0x0700, // Channel 104: 868.027344 Mhz
   0x365,0x08CD, // Channel 105: 869.034241 Mhz
   0x387,0x0000, // Channel 200: 902.999756 Mhz
   0x38b,0x0700, // Channel 201: 907.027344 Mhz
   0x38f,0x0E00, // Channel 202: 911.054932 Mhz
   0x393,0x1534, // Channel 203: 915.082520 Mhz
   0x397,0x1C34, // Channel 204: 919.110107 Mhz
   0x39b,0x2334  // Channel 205: 923.137695 Mhz
};

// rfc_CMD_FS_t.frequency, rfc_CMD_FS_t.fractFreq values for 868Mhz band

uint8_t gChannel;
uint8_t gSubGhzBand;
static RF_Handle gRF;
static RF_Object gRfObject;

int8_t  mLastRSSI;

// Most of the time the tag sends a single packet and receives a single
// packet in response.  
// 
// The exception is the PKT_BLOCK_REQUEST command in this case the tag
// sends one packet and (assuming no packet loss) receives up to 42 packets
// in response.  So we need a command queue with 43 entries.
// rfc_dataEntryGeneral_t gRxDataQueue[BLOCK_MAX_PARTS] __attribute__((aligned(4)));
rfc_dataEntryPointer_t gRxDataQueue[1] __attribute__((aligned(4)));
int8_t gRxPacketLen;
uint8_t gRxBuf[RADIO_MAX_PACKET_LEN + 1];

static dataQueue_t gRxQueue;
static rfc_propRxOutput_t gRxStats;

static void RfCmdCB(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
void LogRF_Event(uint64_t Event);

// Try to receive a packet for up to Wait4Ms milliseconds
// return pointer to data on success
uint8_t *commsRxUnencrypted(uint32_t Wait4Ms)
{
   RF_CmdHandle Err;
   uint8_t *pRet = NULL;

   ratmr_t StartTime;
   ratmr_t EndTime;

//   RF_cmdPropRx.startTime = RF_getCurrentTime();

   gRxPacketLen = 0;
   gRxQueue.pCurrEntry = (uint8_t *) gRxDataQueue;
   gRxQueue.pLastEntry = gRxQueue.pCurrEntry;
   gRxDataQueue[0].status = DATA_ENTRY_PENDING;       // Pending - starting state

   RF_cmdPropRx.endTime = RF_convertMsToRatTicks(Wait4Ms);
   RF_cmdPropRx.pQueue = &gRxQueue;
   RF_cmdPropRx.status = 0;

   LOG("RF_cmdPropRx: \n");
   DUMP_HEX(&RF_cmdPropRx,sizeof(RF_cmdPropRx));

   LOG("gRxQueue: \n");
   DUMP_HEX(&gRxQueue,sizeof(gRxQueue));


   StartTime = RF_getCurrentTime();
   LOG("start %u\n",StartTime);

   Err = RF_runCmd(gRF,
                   (RF_Op*)&RF_cmdPropRx,RF_PriorityNormal,
                   RfCmdCB,
                   0xffffffff
                   // RF_EventLastCmdDone | RF_EventRxOk
                   );

   EndTime = RF_getCurrentTime();
   LOG("end %u, terminationReason 0x%x%08x\n",
       EndTime,
       (uint32_t) (Err >> 16),
       (uint32_t) (Err & 0xffffffff));


   if(Err < 0) {
      LOGE("RF_postCmd failed %d\n",Err);
   }

   if(gRxPacketLen ) {
      mLastRSSI = gRxStats.lastRssi;
      LOG("Received %d byte packet, Rssi %d:\n",gRxPacketLen,mLastRSSI);
      pRet = &gRxDataQueue[0].pData[1];
      DUMP_HEX(pRet,gRxPacketLen);
   }

   return pRet;
}

// Queue a new packet for transmission when commsRxUnencrypted is called.
bool commsTxUnencrypted(const void  *packetP, uint8_t len)
{
   bool bRet = false;   // Assume the best
   RF_cmdPropTx.pPkt = (uint8_t*) packetP;
   RF_cmdPropTx.pktLen = len;
   RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
   RF_EventMask Err = RF_EventCmdAborted | RF_EventCmdPreempted;
   while((Err & RF_EventCmdAborted) && (Err & RF_EventCmdPreempted)) {
   // Re-run if command was aborted due to SW TCXO compensation
      Err = RF_runCmd(gRF,
                      (RF_Op*)&RF_cmdPropTx,RF_PriorityNormal,
                      RfCmdCB,
                      RF_EventRxEntryDone | RF_EventLastCmdDone);
      LOG("terminationReason 0x%x%08x\n",
          (uint32_t) (Err >> 16),
          (uint32_t) (Err & 0xffffffff));
   }
   uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
   if(cmdStatus != 0) {
      LOG("cmdStatus 0x%x\n",cmdStatus);
      bRet = true;
   }
   return bRet;
}

bool radioRxEnable(bool bEnable)
{
   RF_Params rfParams;
   RF_Params_init(&rfParams);
   bool bRet = false;  // assume the best

   do {
      if(RF_cmdPropRx.pQueue == NULL) {
         if((bRet = radioInit())) {
            break;
         }
      }
      if(bEnable) {
         bRet = radioSetChannel(gChannel != 0 ? gChannel : 200);
      }
      else {
         if(gRF != NULL) {
            RF_close(gRF);
            gRF = NULL;
         }
         else {
            LOGE("Radio was not enabled\n");
         }
      }
   } while(false);

   return bRet;
}

void radioRxFlush()
{
}

bool radioInit()
{
   bool bRet = false;   // Assume the best

   LOG("Called\n");
   do {
   // 
      gRxDataQueue[0].pNextEntry = NULL;
      gRxDataQueue[0].status = DATA_ENTRY_PENDING;       // Pending - starting state
      gRxDataQueue[0].config.type = DATA_ENTRY_TYPE_PTR; 
      gRxDataQueue[0].config.lenSz = 0;   // No length indicator byte in data
      gRxDataQueue[0].pData = gRxBuf;
      gRxDataQueue[0].length = RADIO_MAX_PACKET_LEN;
      gRxDataQueue[0].pNextEntry = NULL;


      RF_cmdPropRx.pQueue = &gRxQueue;
        /* Discard ignored packets from Rx queue */
      RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
        /* Discard packets with CRC error from Rx queue */
      RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
      RF_cmdPropRx.maxPktLen = RADIO_MAX_PACKET_LEN;
      RF_cmdPropRx.pktConf.bRepeatOk = 0;
      RF_cmdPropRx.pktConf.bRepeatNok = 0;
      RF_cmdPropRx.pOutput = (uint8_t *)&gRxStats;
      RF_cmdPropRx.startTrigger.triggerType = TRIG_NOW;
//      RF_cmdPropRx.rxConf.bIncludeHdr = 0;   // Do not include length byte in data

        /* Receive operation will end RX_TIMEOUT ms after command starts */
      RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_START;
      RF_cmdPropRx.endTime = RX_TIMEOUT;
   } while(false);

   return bRet;
}

bool radioSetChannel(uint8_t Channel)
{
   uint8_t Band = BAND_UNKNOWN;
   bool bRet = false;   // Assume the best
   uint8_t Index;
   RF_CmdHandle Err;
   LOG("channel %d\n",Channel);

   do {
      if(Channel >= 100 && Channel <= 105) {
      // 866 Mhz
         Index = Channel - 100;
         Band = BAND_868;
      }
      else if(Channel >= 200 && Channel <= 205) {
      // 915 Mhz
         Index = Channel - 200 + 6;
         Band = BAND_915;
      }
      else {
         LOG("%d in an invalid channel",gChannel);
         bRet = true;
         break;
      }
      Index *= 2;
      RF_cmdFs.frequency = gSynthValues[Index++];
      RF_cmdFs.fractFreq = gSynthValues[Index];
      RF_cmdPropRadioDivSetup.centerFreq = RF_cmdFs.frequency;
      LOG("RF_cmdFs.frequency 0x%x RF_cmdFs.fractFreq 0x%x\n",
          RF_cmdFs.frequency,RF_cmdFs.fractFreq);

      if(gRF != NULL) {
         RF_close(gRF);
         gRF = NULL;
         gSubGhzBand = BAND_UNKNOWN;
      }
      RF_Params rfParams;

      RF_Params_init(&rfParams);
      gRF = RF_open(&gRfObject,&RF_prop,
                    (RF_RadioSetup*)&RF_cmdPropRadioDivSetup,
                    &rfParams);
      if(gRF == NULL) {
         LOGE("RF_open failed\n");
         break;
      }
   // Set the frequency
      Err = RF_postCmd(gRF,(RF_Op*)&RF_cmdFs,RF_PriorityNormal,NULL,0);
      if(Err < 0) {
         LOGE("RF_postCmd failed %d\n",Err);
         break;
      }
   } while(false);

   return bRet;
}

static void RfCmdCB(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
   LOG("%u RF_EventMask 0x%x%08x\n",clock_time(),
       (uint32_t) (e >> 16),
       (uint32_t) (e & 0xffffffff));
   LogRF_Event(e);
   if(e & RF_EventRxOk) {
   // Successful RX
      gRxPacketLen = gRxDataQueue[0].pData[0];
      LOG("%u Received %d byte packet\n",clock_time(),gRxPacketLen);
   }
   else if((e & RF_EventLastCmdDone) && !(e & RF_EventRxEntryDone)) {
   }
   else {
   }
}

bool commsTxNoCpy(const void  *packetp)
{
   uint8_t *Bytes = (uint8_t *) packetp;

   LOG("Txlen %d\n",Bytes[0]);
   return commsTxUnencrypted(&Bytes[1],Bytes[0]);
}

const char *EventDesc[] = {
   "RF_EventCmdDone",
   "RF_EventLastCmdDone",
   "RF_EventFGCmdDone",
   "RF_EventLastFGCmdDone",
   "RF_EventTxDone",
   "RF_EventTXAck",
   "RF_EventTxCtrl",
   "RF_EventTxCtrlAck",
   "RF_EventTxCtrlAckAck",
   "RF_EventTxRetrans",
   "RF_EventTxEntryDone",
   "RF_EventTxBufferChange",  // (1 << 11)
   NULL, // (1 << 12)
   NULL, // (1 << 13)
   "RF_EventPaChanged",
   "RF_EventSamplesEntryDone",
   "RF_EventRxOk",
   "RF_EventRxNOk",
   "RF_EventRxIgnored",
   "RF_EventRxEmpty",
   "RF_EventRxCtrl",
   "RF_EventRxCtrlAck",
   "RF_EventRxBufFull",
   "RF_EventRxEntryDone",
   "RF_EventDataWritten",
   "RF_EventNDataWritten",
   "RF_EventRxAborted",
   "RF_EventRxCollisionDetected",   // (1 << 27)
   NULL,                            // (1 << 28)
   "RF_EventModulesUnlocked",       // (1 << 29)
   NULL,                            // (1 << 30)
   "RF_EventInternalError"         // (1 << 31)
};

void LogRF_Event(uint64_t Event)
{
   int i;
   uint64_t Mask = 1;
   bool bFirst = true;

   for(i = 0; i < 64; i++) {
      if(Event & Mask) {
         if(bFirst) {
            bFirst = false;
         }
         else {
            LOG_RAW(" | ");
         }

         if(i <= 31 && EventDesc[i] != NULL) {
            LOG_RAW("%s",EventDesc[i]);
         }
         else {
            LOG_RAW("bit %d",i);
         }
      }
      Mask <<= 1;
   }
   LOG_RAW("\n");
}
