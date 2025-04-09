#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "powermgt.h"
#include "main.h"
#include "board.h"
#include "syncedproto.h"
#include "oepl-proto.h"
#include "oepl-definitions.h"
#include "logging.h"

uint16_t dataReqAttemptArr[POWER_SAVING_SMOOTHING] = {0}; // Holds the amount of attempts required per data_req/check-in
uint8_t dataReqAttemptArrayIndex = 0;
uint8_t dataReqLastAttempt = 0;
uint16_t nextCheckInFromAP = 0;
uint8_t scanAttempts = 0;

uint16_t longDataReqCounter = 0;
uint16_t voltageCheckCounter = 0;


uint64_t time_ms = 0;
uint32_t time_overflow = 0;

uint32_t getMillis()
{
   return ClockP_getSystemTicks() / 100;
}

void initPowerSaving(const uint16_t initialValue)
{
    for (uint8_t c = 0; c < POWER_SAVING_SMOOTHING; c++)
    {
        dataReqAttemptArr[c] = initialValue;
    }
}

// t == sleep time in milliseconds, 0 means forever
void doSleep(uint32_t t)
{
   LOG("Sleep for %u...",t);
#ifdef DEBUG_MAX_SLEEP
   if(t > DEBUG_MAX_SLEEP) {
      t = DEBUG_MAX_SLEEP;
   }
#endif
   if(t > (0xffffffff / 1000)) {
      ClockP_sleep(t / 1000);
   }
   else {
      ClockP_usleep(t * 1000);
   }
   LOG("\n");
}

uint32_t getNextScanSleep(const bool increment)
{
    if (increment)
    {
        if (scanAttempts < 255)
            scanAttempts++;
    }

    if (scanAttempts < INTERVAL_1_ATTEMPTS)
    {
        return INTERVAL_1_TIME;
    }
    else if (scanAttempts < (INTERVAL_1_ATTEMPTS + INTERVAL_2_ATTEMPTS))
    {
        return INTERVAL_2_TIME;
    }
    else
    {
        return INTERVAL_3_TIME;
    }
}

void addAverageValue()
{
    uint16_t curval = INTERVAL_AT_MAX_ATTEMPTS - INTERVAL_BASE;
    curval *= dataReqLastAttempt;
    curval /= DATA_REQ_MAX_ATTEMPTS;
    curval += INTERVAL_BASE;
    dataReqAttemptArr[dataReqAttemptArrayIndex % POWER_SAVING_SMOOTHING] = curval;
    dataReqAttemptArrayIndex++;
}

uint16_t getNextSleep()
{
    uint16_t avg = 0;
    for (uint8_t c = 0; c < POWER_SAVING_SMOOTHING; c++)
    {
        avg += dataReqAttemptArr[c];
    }
    avg /= POWER_SAVING_SMOOTHING;
    return avg;
}
