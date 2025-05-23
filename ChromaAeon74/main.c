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
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/dpl/ClockP.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)

/* Driver configuration */
#include "ti/drivers/gpio/GPIOCC26XX.h"
#include "ti_drivers_config.h"
#include "epd.h"
#include "img.h"
#include "main.h"
#include "img.h"
#include "board.h"
#include "radio.h"
#include "syncedproto.h"
#include "oepl-proto.h"
#include "oepl-definitions.h"
#include "powermgt.h"
#include "drawing.h"
#include "eeprom.h"
#include "logging.h"

uint8_t wakeUpReason = WAKEUP_REASON_FIRSTBOOT;
int8_t  temperature;
uint16_t  batteryVoltage;
bool  lowBattery;
bool noApShown;

uint8_t capabilities;

uint8_t gTempBuf[TEMP_BUF_SIZE];


void InitSN(void);
uint8_t detectAP(const uint8_t channel);
uint8_t channelSelect(uint8_t rounds);
void GetTempAndBattV(void);

void bbTest();

void *mainThread(void *arg0)
{
   InitLogging();
   ALOG("ChromaAeon74 Ver %d compiled " __DATE__ " " __TIME__ "\n",FW_VERSION);

   bbTest();
   InitSN();
   if(radioInit()) {
      LOG("radioInit failed\n");
   }

   initializeProto();
   currentChannel = channelSelect(1);

   if(currentChannel) {
      LOG("AP Found\n");
#if 0
      epd_display("AP Found", batteryVoltage, ownMacString, 1);
      initPowerSaving(INTERVAL_BASE);
#endif
      doSleep(5000UL);
   }
   else {
      LOG("No AP found\n");
#if 0
      epd_display("No AP Found", batteryVoltage, ownMacString, 1);
      initPowerSaving(INTERVAL_AT_MAX_ATTEMPTS);
#endif
      noApShown = true;
      doSleep(120000UL);
   }

   while(1) {
      wdt10s();
      GetTempAndBattV();
      if(currentChannel) {
         struct AvailDataInfo *avail;
         avail = getAvailDataInfo();
      // avail = getShortAvailDataInfo();
         addAverageValue();

         if(avail == NULL) {
            LOG("No data\n");
            nextCheckInFromAP = 0; // let the power-saving algorithm determine the next sleep period
         }
         else {
            nextCheckInFromAP = avail->nextCheckIn;
         // got some data from the AP!
            if(avail->dataType != DATATYPE_NOUPDATE) {
               LOG("Got data transfer\n");
            // data transfer
               if(processAvailDataInfo(avail)) {
               // succesful transfer, next wake time is determined by the NextCheckin;
               }
               else {
               // failed transfer, let the algorithm determine next sleep interval (not the AP)
                  nextCheckInFromAP = 0;
               }
            }
            else {
               LOG("No data transfer\n");
               wakeUpReason = WAKEUP_REASON_TIMED; // Only one successfully AP communication we can have timed wakeups
                                           // no data transfer, just sleep.
            }
         }

         uint16_t nextCheckin = getNextSleep();
         longDataReqCounter += nextCheckin;
         if(nextCheckin == INTERVAL_AT_MAX_ATTEMPTS) {
         // disconnected, obviously...
            currentChannel = 0;
         }

         doSleep(40 * 1000UL);
          /*// if the AP told us to sleep for a specific period, do so.
          if (nextCheckInFromAP)
          {
             doSleep(nextCheckInFromAP * 60000UL);
          }
          else
          {
             doSleep(getNextSleep() * 1000UL);
          }*/
      }
      else {
      // We sacrifice 10ms here to show a basic LED status as the scan itself takes more than a second
         WaitMs(10);
         currentChannel = channelSelect(1);

         if(!currentChannel) {
            wdt60s();
            if(!noApShown) {
               noApShown = true;
               if(curImgSlot != 0xFF) {
                  drawOnOffline(0);
                  drawImageFromEeprom(curImgSlot);
                  drawOnOffline(1);
               }
               else {
#if 0
                  epd_display("No AP", batteryVoltage, ownMacString, 1);
#endif
               }
            }
         }
      // did we find a working channel?
         if(currentChannel) {
         // now associated!
            noApShown = false;
            LOG("AP Found\n");
            if(curImgSlot != 0xFF) {
               drawOnOffline(1);
               drawImageFromEeprom(curImgSlot);
            }
            else {
#if 0
               epd_display("AP Found", batteryVoltage, ownMacString, 1);
#endif
            }
            scanAttempts = 0;
            wakeUpReason = WAKEUP_REASON_NETWORK_SCAN;
            initPowerSaving(INTERVAL_BASE);
            doSleep(40 * 1000UL);
         }
         else {
         // still not associated
            doSleep(15 * 60 * 1000UL);
         }
      }
   }

   return(NULL);
}

void InitSN()
{
   char SN[SN_LEN];

   memcpy(SN,(void *) SN_OFFSET,SN_LEN);

   if(SN[0] != SN_CHAR1 || SN[1] != SN_CHAR2 || SN[6] != SN_REV_CHAR) {
      LOG("Invalid SN:\n");
      DUMP_HEX(SN,SN_LEN);
      LOG("Setting default SN\n");
      memset(SN,0,sizeof(SN));
      SN[0] = SN_CHAR1;
      SN[1] = SN_CHAR2;
      SN[6] = SN_REV_CHAR;
   }
   mSelfMac[7] = 0x44;
   mSelfMac[6] = 0x67;
   mSelfMac[3] = SN[2];
   mSelfMac[2] = SN[3];
   mSelfMac[1] = SN[4];
   mSelfMac[0] = SN[5];
// Since the first 2 characters are known to be upper case ASCII subtract
// 'A' from the value to convert it from a 7 bit value to a 5 bit value.
// 
// This allows us to use the extra 3 bits to convey something else like the
// hardware variation.
// 
// i.e. Multiple incompatible hardware variations that need different FW 
// images, but otherwise are compatible can share a single HWID and the 
// AP can parse the "extra" bits in the MAC address to select the 
// correct OTA image.

   mSelfMac[5] = SN[0] - 'A';
   mSelfMac[4] = SN[1] - 'A';
#ifdef HW_VARIANT
   mSelfMac[5] |= HW_VARIANT << 5;
   LOG("HW variant %d\n",HW_VARIANT);
#endif

   LOG("SN: %c%c%02x%02x%02x%02x%c\n",
       SN[0],SN[1],SN[2],SN[3],SN[4],SN[5],SN[6]);

   LOG("MAC: ");
   DUMP_HEX(mSelfMac,sizeof(mSelfMac));
}


// Return 0 if no APs are found
// 
// Initally we search for APs on both bands once we have found
// one we know what band to search and only search that band to
// avoid (further) out of band operation.
uint8_t channelSelect(uint8_t rounds) 
{
   const uint8_t ChannelList[] = {
      200,100,201,101,202,102,203,103,204,104,205,105
   };
   int8_t BestRssi = -128;
   uint8_t BestChannel = 0;
   uint8_t i;

   while(rounds-- > 0) {
      for(i = 0; i < sizeof(ChannelList); i++) {
         if(gSubGhzBand == BAND_915 && ChannelList[i] < 200) {
         // Using 915 band and this channel is in 868, skip it
         }
         else if(gSubGhzBand == BAND_868 && ChannelList[i] >= 200) {
         // Using 868 band and this channel is in 915, skip it
         }
         else {
            if(radioSetChannel(ChannelList[i])) {
               break;
            }
            if(detectAP(ChannelList[i])) {
               LOG("Chan %d RSSI %d\n",ChannelList[i],mLastRSSI);
               if(BestRssi < mLastRSSI) {
                  BestRssi = mLastRSSI;
                  BestChannel = ChannelList[i];
               }
            }
         }
      }
   }

   if(BestChannel) {
      LOG("Using ch %d\n",BestChannel);
      if(BestChannel >= 200) {
         gSubGhzBand = BAND_915;
      }
      else {
         gSubGhzBand = BAND_868;
      }
      if(gChannel != BestChannel) {
         gChannel = BestChannel;
      // TODO: save gChannel in flash
      }
   }
   else {
      LOG("No AP found\n");
   }
   return BestChannel;
}


void drawOnOffline(uint8_t state)
{
   LOG("State %d\n",state);
}

void wdt10s(void)
{
}

void wdt60s(void)
{
}

void GetTempAndBattV(void)
{
   AONBatMonEnable();
   VLOG("Waiting for New Temp MeasureReady...");
   while(!AONBatMonNewTempMeasureReady())
      ;
   VLOG("\n");
   temperature = (int8_t) AONBatMonTemperatureGetDegC();
   VLOG("Temp %d C\n",temperature);
   VLOG("Waiting for New Battery MeasureReady...");
   while(!AONBatMonNewBatteryMeasureReady())
      ;
   VLOG("\n");
   uint32_t Temp = AONBatMonBatteryVoltageGet();
   batteryVoltage = (Temp & 0xff) * 1000 / 256;
   batteryVoltage += (Temp >> 8) * 1000;
   VLOG("BatV %d.%03d\n",batteryVoltage / 1000, batteryVoltage % 1000);
   AONBatMonDisable();
}
