/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>

/* TI Drivers */
#include <ti/display/Display.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>

#include "oepl-definitions.h"
#include "oepl-proto.h"

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "ti_drivers_config.h"

/* Application Header files */
#include "RFQueue.h"
#include <ti_radio_config.h>

/***** Defines *****/
/* Packet TX/RX Configuration */
#define PAYLOAD_LENGTH      255
/* Set packet interval to 1000ms */
#define PACKET_INTERVAL     (uint32_t)(1100000)
#define RX_TIMEOUT          (uint32_t)(1000000)
/* NOTE: Only two data entries supported at the moment */
#define NUM_DATA_ENTRIES    2
/* The Data Entries data field will contain:
 * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
 * Max 30 payload bytes
 * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */
#define NUM_APPENDED_BYTES  2

#define LOG(format, ... ) Display_printf(gDisplayHandle, 0, 0,format,## __VA_ARGS__)

/* Log radio events in the callback */
//#define LOG_RADIO_EVENTS

/***** Prototypes *****/
static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary
 * (requirement from the RF core)
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(rxDataEntryBuffer, 4)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported
#endif //defined(__TI_COMPILER_VERSION__)

/* Receive Statistics */
static rfc_propRxOutput_t rxStatistics;

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

static uint8_t txPacket[PAYLOAD_LENGTH];
static uint8_t rxPacket[PAYLOAD_LENGTH + NUM_APPENDED_BYTES - 1];
static uint16_t seqNumber;

static volatile bool bRxSuccess = false;

#ifdef LOG_RADIO_EVENTS
static volatile RF_EventMask eventLog[32];
static volatile uint8_t evIndex = 0;
#endif // LOG_RADIO_EVENTS

Display_Handle gDisplayHandle;

// rfc_CMD_FS_t.frequency, rfc_CMD_FS_t.fractFreq values for 868Mhz band
const uint16_t g868SynthValues[6][2] = {
   {0x360,0x0000},   // Channel 100 / CHANNR 0:  863.999756 Mhz
   {0x361,0x01CD},   // Channel 101 / CHANNR 3:  865.006653 Mhz
   {0x362,0x0367},   // Channel 102 / CHANNR 6:  866.013550 Mhz
   {0x363,0x0534},   // Channel 103 / CHANNR 9:  867.020447 Mhz
   {0x364,0x0700},   // Channel 104 / CHANNR 12: 868.027344 Mhz
   {0x365,0x08CD}   // Channel 105 / CHANNR 15: 869.034241 Mhz
};

// rfc_CMD_FS_t.frequency, rfc_CMD_FS_t.fractFreq values for 868Mhz band
const uint16_t g915SynthValues[6][2] = {
   {0x387,0x0000},   // Channel 200 / CHANNR 0:  902.999756 Mhz
   {0x38b,0x0700},   // Channel 201 / CHANNR 12: 907.027344 Mhz
   {0x38f,0x0E00},   // Channel 202 / CHANNR 24: 911.054932 Mhz
   {0x393,0x1534},   // Channel 203 / CHANNR 36: 915.082520 Mhz
   {0x397,0x1C34},   // Channel 204 / CHANNR 48: 919.110107 Mhz
   {0x39b,0x2334}   // Channel 205 / CHANNR 60: 923.137695 Mhz
};

uint8_t gChannel = 99;

static uint8_t CreatePingPacket(void);
void CheckPing(uint8_t *pRx,uint8_t PacketLength);

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    uint32_t curtime;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

#if 1
    // Sets up display
    Display_init();
    gDisplayHandle = Display_open(Display_Type_UART, NULL);
    if (gDisplayHandle == NULL)
    {
        // Display_open() failed
        return (NULL);
    } else {
        LOG("Hi from ChromaAeon74! :D");
    }
#endif


    if(RFQueue_defineQueue(&dataQueue,
                           rxDataEntryBuffer,
                           sizeof(rxDataEntryBuffer),
                           NUM_DATA_ENTRIES,
                           PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */
    RF_cmdPropTx.pPkt = txPacket;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;
    RF_cmdPropTx.pNextOp = (rfc_radioOp_t *)&RF_cmdPropRx;
    /* Only run the RX command if TX is successful */
    RF_cmdPropTx.condition.rule = COND_STOP_ON_FALSE;

    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = PAYLOAD_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 0;
    RF_cmdPropRx.pktConf.bRepeatNok = 0;
    RF_cmdPropRx.pOutput = (uint8_t *)&rxStatistics;
    /* Receive operation will end RX_TIMEOUT ms after command starts */
    RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_PREVEND;
    RF_cmdPropRx.endTime = RX_TIMEOUT;

    int Pings = 4;
    bool bRfOpen = false;
    while(1)
    {

       if(Pings++ > 2) {
       // Next channel
          Pings = 0;
          gChannel++;
          if(gChannel == 106) {
             if(bRfOpen) {
                bRfOpen = false;
                RF_close(rfHandle);
             }
             gChannel = 200;

          }
          else if(gChannel == 206) {
             gChannel = 100;
          }

          if(gChannel >= 100 && gChannel <= 105) {
          // 866 Mhz
             RF_cmdPropRadioDivSetup.centerFreq = g868SynthValues[gChannel-100][0];
             RF_cmdFs.frequency = g868SynthValues[gChannel-100][0];
             RF_cmdFs.fractFreq = g868SynthValues[gChannel-100][1];
          }
          else if(gChannel >= 200 && gChannel <= 205) {
          // 915 Mhz
             RF_cmdPropRadioDivSetup.centerFreq = g868SynthValues[gChannel-200][0];
             RF_cmdFs.frequency = g915SynthValues[gChannel-200][0];
             RF_cmdFs.fractFreq = g915SynthValues[gChannel-200][1];
          }
          else {
             LOG("%d in an invalid channel",gChannel);
             break;
          }

          if(bRfOpen) {
             RF_close(rfHandle);
          }
          bRfOpen = true;
          /* Request access to the radio */
          rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

          /* Set the frequency */
          RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
          LOG("Pinging on channel %d",gChannel);
       }

        /* Create packet with incrementing sequence number and random payload */

       RF_cmdPropTx.pktLen = CreatePingPacket();
       // LOG("RF_cmdPropTx.pktLen %d",RF_cmdPropTx.pktLen);

        /* Set absolute TX time to utilize automatic power management */
       /* Get current time */
       curtime = RF_getCurrentTime();
        curtime += PACKET_INTERVAL;
        RF_cmdPropTx.startTime = curtime;

        /* Transmit a packet and wait for its echo.
         * - When the first of the two chained commands (TX) completes, the
         * RF_EventCmdDone event is raised but not RF_EventLastCmdDone
         * - The RF_EventLastCmdDone in addition to the RF_EventCmdDone events
         * are raised when the second, and therefore last, command (RX) in the
         * chain completes
         * -- If the RF core successfully receives the echo it will also raise
         * the RF_EventRxEntryDone event
         * -- If the RF core times out while waiting for the echo it does not
         * raise the RF_EventRxEntryDone event
         */
        RF_EventMask terminationReason = RF_EventCmdAborted | RF_EventCmdPreempted;
        while(( terminationReason & RF_EventCmdAborted ) && ( terminationReason & RF_EventCmdPreempted ))
        {
            // Re-run if command was aborted due to SW TCXO compensation
            terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal,
                                          echoCallback, (RF_EventCmdDone | RF_EventRxEntryDone |
                                          RF_EventLastCmdDone));
        }

        // LOG("terminationReason 0x%lx",terminationReason);

        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        // LOG("cmdStatus 0x%x",cmdStatus);
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }
    }
    while(1);
}

static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
#ifdef LOG_RADIO_EVENTS
    eventLog[evIndex++ & 0x1F] = e;
#endif// LOG_RADIO_EVENTS

    // LOG("echoCallback: RF_EventMask 0x%lx",e);

    if((e & RF_EventCmdDone) && !(e & RF_EventLastCmdDone))
    {
        /* Successful TX */
    }
    else if(e & RF_EventRxEntryDone)
    {
        /* Successful RX */
        bRxSuccess = true;

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &(currentDataEntry->data):
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte
         */
        packetLength      = *(uint8_t *)(&(currentDataEntry->data));
        packetDataPointer = (uint8_t *)(&(currentDataEntry->data) + 1);

        /* Copy the payload + status byte to the rxPacket variable */
        memcpy(rxPacket, packetDataPointer, (packetLength + 1));
        CheckPing(rxPacket,packetLength);

        LOG("Received %d byte packet",packetLength);

        RFQueue_nextEntry();
    }
    else if((e & RF_EventLastCmdDone) && !(e & RF_EventRxEntryDone))
    {
        if(bRxSuccess == true)
        {
            /* Received packet successfully but RX command didn't complete at
             * the same time RX_ENTRY_DONE event was raised. Reset the flag
             */
            bRxSuccess = false;
        }
        else
        {
            /* RX timed out */

        }
    }
    else
    {
        /* Error Condition: set both LEDs */

    }
}

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
    struct MacFrameBcast *txframe = (struct MacFrameBcast *)txPacket;
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

static bool pktIsUnicast(const void *buffer)
{
    const struct MacFcs *fcs = buffer;
    if ((fcs->frameType == 1) && (fcs->destAddrType == 2) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 0))
    {
        return false;
    }
    else if ((fcs->frameType == 1) && (fcs->destAddrType == 3) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 1))
    {
        // normal frame
        return true;
    }
    // unknown type...
    return false;
}

void CheckPing(uint8_t *pRx,uint8_t PacketLength)
{
   struct MacFrameBcast *Rxframe = (struct MacFrameBcast *)pRx;


}
