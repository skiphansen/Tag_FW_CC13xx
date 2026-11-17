#ifndef _MAIN_H_
#define _MAIN_H_

// Chroma 74H OEPL build (counterpart of ChromaAeon74/main.h)
#include <stdbool.h>
#include <stdint.h>
#include "cc13xx.h"

// No DEBUG_MAX_SLEEP (powermgt.c caps every sleep with it): the AP holds new
// content until the check-in it expects from the sleep it offered, so a tag
// that wakes up early gains nothing. For fast turnaround use the AP's
// "Shorten latency during config" with its web UI open.

// Reported to the AP, which picks the image format from it: G5 once the
// version reaches the tagtype's "g5_compression" value (hex), zlib once it
// reaches "zlib_compression" (zlib wins when both apply). 2 = raw images
// only, 3 = G5 (provisional hwType 0x87), 4 = OTA through bootloader_CC1310,
// 5 = boot screens, 6 = quick re-check after a boot check-in, 7 = reset
// records out of the bootloader's RAM, 8 = zlib, hwType 0x81 (81.json:
// zlib_compression "3"). Can be overridden for OTA tests
// (EXTRA_DEFINES=-DFW_VERSION=...).
#ifndef FW_VERSION
#define FW_VERSION         0x0008
#endif

#define TEMP_BUF_SIZE      256
extern uint8_t gTempBuf[TEMP_BUF_SIZE];
extern uint8_t mSelfMac[8];

void wdt10s(void);
void wdt60s(void);
void wdtSet(uint32_t Ms);   // watchdog period Ms, max 2863 s

extern uint8_t wakeUpReason;
extern int8_t  temperature;
extern uint16_t  batteryVoltage;    // reported to the AP
extern uint16_t  batteryRestMv;     // last BATMON reading, never the refresh minimum
extern bool  lowBattery;
extern uint8_t capabilities;

// epd_74h.c
void drawImageAtAddress(uint32_t addr, uint8_t lut);

#endif   // _MAIN_H_
