/*
 * E-paper driver for the Chroma 74H (800x480 BWY, CC1310).
 *
 * The panel has no D/C line: it takes 3-wire SPI with 9 bit frames, the D/C
 * flag first (0 = command, 1 = data). Sent on SSI1 as the stock firmware
 * does (EPD_HW_SPI), or bit banged as on the Aeon 74.
 *
 * The command sequence is not hardcoded. Like the stock firmware, this
 * runs the factory command script stored in SPI flash at 0x2000 (magic
 * 0xBEEFFACE, format 04 03). Record types, power/reset timing and the
 * hidden register table are from the stock firmware, see
 * ../chroma74h_reverse_engineering/docs/epd.md and CLAUDE.md.
 */
#include "oepl_board.h"

#ifdef CHROMA_74H
#include <string.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/gpio.h)
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)
#include <ti/drivers/dpl/ClockP.h>
#ifdef EPD_HW_SPI
#include DeviceFamily_constructPath(driverlib/ssi.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#endif

#include "Board.h"
#include "cc13xx.h"
#include "eeprom.h"
#include "epd_74h.h"
#include "oepl-definitions.h"
#include "logging.h"
#include "font8x8.h"

#ifdef OEPL_74H
#include "main.h"
#define EpdWdt(Ms)   wdtSet(Ms)
#define EPD_TEMPERATURE temperature   // AONBatMon, whole degrees C
#else
#define EpdWdt(Ms)
#define EPD_TEMPERATURE 25
#endif

#define EPD_PWR     CHROMA_EPD_PWR    // active low
#define EPD_BUSY    CHROMA_EPD_BUSY   // LOW while busy (measured on hardware)
#define EpdBusy()   (GPIO_readDio(EPD_BUSY) == 0)
#define EPD_CS      CHROMA_EPD_CS
#define EPD_CLK     CHROMA_EPD_CLK
#define EPD_SDI     CHROMA_EPD_SDI
#define EPD_RST     CHROMA_EPD_RST

#define SCRIPT_MAGIC       0xBEEFFACE
#define SCRIPT_MAX_LEN     0x200
// Script body: offsets in it are relative to SCRIPT_BODY. It starts with
// u32 offsets of segment 1, the temperature band table and segment 3. Band
// entries are 9 bytes: int8 threshold in degrees C, u32 offset of segment 2,
// 4 unused; highest threshold first, ended by a threshold below -99. A draw
// runs segment 1, segment 2 of the first band whose threshold is <= the
// temperature, then segment 3 (stock: RE project epd.md 6b). On the bench
// 74H segments 1 and 2 are empty and segment 3 starts at 0x47.
#define SCRIPT_BODY        0x2a
#define SCRIPT_BAND_SIZE   9
#define SCRIPT_BAND_END    (-99)
// Used if the body header doesn't parse
#define SCRIPT_RECORDS     0x47

// A full BWY refresh measured 51.8 s at room temperature (2026-11-16)
#define BUSY_TIMEOUT_MS    120000
// BUSY asserts a short time after the command that starts an operation; the
// stock firmware waits for the edge, not the level.
#define BUSY_ASSERT_MS     500
// Used if BUSY never rises on the refresh wait, so a refresh isn't cut off
#define REFRESH_FALLBACK_MS 30000
// Two rows per flash read; the layer is applied row by row
#define CHUNK_SIZE         (2 * EPD_ROW_BYTES)

// Bring-up diagnostics, readable over JTAG (see oepl_CC1310.map for the address)
#define EPD_DIAG_WAITS     8
struct {
   uint32_t Magic;            // 0xEDD1A600 + number of waits recorded
   uint16_t AssertMs[EPD_DIAG_WAITS];  // 0xffff: BUSY never asserted
   uint16_t BusyMs[EPD_DIAG_WAITS];
   uint8_t  Record[EPD_DIAG_WAITS];    // script record type of the wait
   uint8_t  Result;           // 1 = ok, 2 = failed
   uint16_t PlaneMs[2];       // upload time of the B/W and yellow plane
   // Supply (VDDS) during the busy waits of the last draw, from AON BATMON:
   // the panel's behaviour depends on it (CLAUDE.md), and with a resistor in
   // series with the supply the drop gives the current
   uint16_t MinMv;
   uint16_t MaxMv;
   uint16_t Samples;          // fresh BATMON readings
} gEpdDiag;

// Lowest supply seen during the last draw, 0 = none since it was read
uint16_t gEpdRefreshMinMv;

static void EpdVddsSample(void)
{
   uint32_t Raw;
   uint16_t Mv;

   if(!AONBatMonNewBatteryMeasureReady()) {
      return;
   }
   Raw = AONBatMonBatteryVoltageGet();    // clears the ready flag
   Mv = (Raw >> 8) * 1000 + (Raw & 0xff) * 1000 / 256;
   if(gEpdDiag.Samples == 0 || Mv < gEpdDiag.MinMv) {
      gEpdDiag.MinMv = Mv;
   }
   if(Mv > gEpdDiag.MaxMv) {
      gEpdDiag.MaxMv = Mv;
   }
   gEpdDiag.Samples++;
}

// Event trace for hangs, kept through a reset for a JTAG read (gEpdTrace in
// the .map): Ev[] is a ring, Idx counts events; each entry is
// (milliseconds << 8) | event.
#pragma NOINIT(gEpdTrace)
struct {
   uint32_t Idx;
   uint32_t Ev[16];
} gEpdTrace;

#define EpdTrace(e) \
   gEpdTrace.Ev[gEpdTrace.Idx++ & 15] = \
      ((ClockP_getSystemTicks() * ClockP_tickPeriod / 1000) << 8) | (e)
#define TR_RECORD          0x00  // | record type (< 0x10)
#define TR_WAIT_ASSERTED   0x20
#define TR_WAIT_DONE       0x21
#define TR_LOG_START       0x22
#define TR_LOG_DONE        0x23
#define TR_DRAW_START      0x30
#define TR_DRAW_DONE       0x31

// Hidden register writes the stock firmware sends after every reset
// (stock table at 0xBB30; the factory script repeats them).
static const uint8_t gHiddenRegs[] = {
   0x7c,0x00, 0x60,0xa5, 0x93,0x18, 0x73,0x07,
   0x92,0x08, 0xa8,0x3a, 0x88,0x06, 0x68,0x10
};

#ifdef EPD_HW_SPI
// SSI1, 9 bit frames, mode 0 (inferred: the stock SSI1 clock/mode setup isn't
// located yet, RE project epd.md section 1). CS (DIO2) is driven by hand, as
// stock does. CLK and SDI belong to SSI1 only while the panel is powered.
#ifndef EPD_SPI_HZ
#define EPD_SPI_HZ         4000000
#endif

// EpdPower(false) runs twice per draw (the script's panel off record, then
// EpdDrawFromFlash()), and SSI1 registers fault once its clock is released:
// open and close only once.
static bool gSpiOpen;

static void EpdSpiOpen(void)
{
   if(gSpiOpen) {
      return;
   }
   gSpiOpen = true;
   Power_setDependency(PowerCC26XX_PERIPH_SSI1);
   IOCPortConfigureSet(EPD_CLK,IOC_PORT_MCU_SSI1_CLK,IOC_STD_OUTPUT);
   IOCPortConfigureSet(EPD_SDI,IOC_PORT_MCU_SSI1_TX,IOC_STD_OUTPUT);
   SSIDisable(SSI1_BASE);
   SSIConfigSetExpClk(SSI1_BASE,48000000,SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,
                      EPD_SPI_HZ,9);
   SSIEnable(SSI1_BASE);
}

static void EpdSpiClose(void)
{
   if(!gSpiOpen) {
      return;
   }
   gSpiOpen = false;
   while(SSIBusy(SSI1_BASE));
   SSIDisable(SSI1_BASE);
// back to GPIO (output enable is still set from the board init)
   IOCPortConfigureSet(EPD_CLK,IOC_PORT_GPIO,IOC_STD_OUTPUT);
   IOCPortConfigureSet(EPD_SDI,IOC_PORT_GPIO,IOC_STD_OUTPUT);
   Power_releaseDependency(PowerCC26XX_PERIPH_SSI1);
}

static void EpdWrite9(uint16_t Data)
{
   uint32_t Dummy;

   SSIDataPut(SSI1_BASE,Data);
// Nothing is read: keep the RX FIFO from overflowing
   while(SSIDataGetNonBlocking(SSI1_BASE,&Dummy));
}

// Raise CS after the last frame has left the shift register
static void EpdCsHigh(void)
{
   uint32_t Dummy;

   while(SSIBusy(SSI1_BASE));
   while(SSIDataGetNonBlocking(SSI1_BASE,&Dummy));
   GPIO_setDio(EPD_CS);
}
// SSI1 loses its configuration in standby (the peripheral power domain goes
// off; TI's drivers re-init through Power notifications, this code doesn't),
// so it is closed before every sleep and reopened on demand. Left open over
// the refresh wait, the next command stuck in a dead FIFO and the watchdog
// fired.
static void EpdSleep(uint32_t Ms)
{
   EpdSpiClose();
   WaitMs(Ms);
}

static void EpdCsLow(void)
{
   EpdSpiOpen();
   GPIO_clearDio(EPD_CS);
}
#else
static void EpdWrite9(uint16_t Data)
{
   uint16_t Mask = 0x100;

   while(Mask != 0) {
      GPIO_writeDio(EPD_SDI,(Data & Mask) ? 1 : 0);
      GPIO_setDio(EPD_CLK);
      GPIO_clearDio(EPD_CLK);
      Mask >>= 1;
   }
}

#define EpdCsHigh()  GPIO_setDio(EPD_CS)
#define EpdCsLow()   GPIO_clearDio(EPD_CS)
#define EpdSleep(Ms) WaitMs(Ms)
#endif

static void EpdCmd(uint8_t Cmd,const uint8_t *pData,int Len)
{
   EpdCsLow();
   EpdWrite9(Cmd);
   while(Len-- > 0) {
      EpdWrite9(0x100 | *pData++);
   }
   EpdCsHigh();
}

// Wait for BUSY to assert, then release. Returns true on timeout.
static bool EpdWaitBusy(uint8_t Record)
{
   uint32_t Asserted = 0;
   uint32_t Waited = 0;
   bool bRet = false;
   int i = gEpdDiag.Magic & 0xff;

   EpdWdt(BUSY_ASSERT_MS + REFRESH_FALLBACK_MS + BUSY_TIMEOUT_MS + 10000);
   AONBatMonEnable();
   while(!EpdBusy() && Asserted < BUSY_ASSERT_MS) {
      EpdSleep(1);
      Asserted++;
   }
   EpdTrace(TR_WAIT_ASSERTED);

   if(Asserted >= BUSY_ASSERT_MS) {
      ELOG("BUSY never asserted\n");
      if(Record == 5) {
      // the refresh wait: don't cut the refresh short
         EpdSleep(REFRESH_FALLBACK_MS);
      }
   }
   else {
      while(EpdBusy()) {
         if(Waited >= BUSY_TIMEOUT_MS) {
            ELOG("BUSY timeout\n");
            bRet = true;
            break;
         }
         EpdVddsSample();
         EpdSleep(10);
         Waited += 10;
      }
   }
   EpdVddsSample();
   AONBatMonDisable();
   EpdTrace(TR_WAIT_DONE);
   EpdTrace(TR_LOG_START);
   ALOG("busy: asserted after %u ms, %u ms busy\n",Asserted,Waited);
   EpdTrace(TR_LOG_DONE);

   if(i < EPD_DIAG_WAITS) {
      gEpdDiag.AssertMs[i] = Asserted >= BUSY_ASSERT_MS ? 0xffff : Asserted;
      gEpdDiag.BusyMs[i] = Waited;
      gEpdDiag.Record[i] = Record;
      gEpdDiag.Magic++;
   }
   return bRet;
}

static void EpdPower(bool bOn)
{
   if(bOn) {
      EpdSleep(5);
      GPIO_clearDio(EPD_RST);
      GPIO_clearDio(EPD_PWR);    // power on
      EpdSleep(25);
      GPIO_setDio(EPD_CS);
   }
   else {
#ifdef EPD_HW_SPI
      EpdSpiClose();
#endif
      GPIO_setDio(EPD_PWR);      // power off
      GPIO_clearDio(EPD_RST);
      GPIO_clearDio(EPD_CS);
      GPIO_clearDio(EPD_CLK);
      GPIO_clearDio(EPD_SDI);
   }
}

static void EpdReset(void)
{
   int i;

   GPIO_clearDio(EPD_RST);
   EpdSleep(2);
   GPIO_setDio(EPD_RST);
   GPIO_setDio(EPD_CS);
   EpdSleep(5);
   for(i = 0; i < sizeof(gHiddenRegs); i += 2) {
      EpdCmd(0xf8,&gHiddenRegs[i],2);
   }
}

static const EpdLayer *gLayer;

void EpdSetLayer(const EpdLayer *pLayer)
{
   gLayer = pLayer;
}

// Ink W pixels from X on in one row of a plane, in panel polarity: B/W plane
// 1 = white, yellow plane 0 = yellow
static void LayerPixels(uint8_t *pRow,int X,int W,int Plane,uint8_t Ink)
{
   bool bSet = Plane == 0 ? Ink != EPD_INK_BLACK : Ink != EPD_INK_YELLOW;
   uint8_t Mask;

   if(X + W > EPD_WIDTH) {
      W = EPD_WIDTH - X;
   }
   for( ; W > 0; W--, X++) {
      Mask = 0x80 >> (X & 7);
      if(bSet) {
         pRow[X >> 3] |= Mask;
      }
      else {
         pRow[X >> 3] &= ~Mask;
      }
   }
}

static void LayerRow(uint8_t *pRow,int Row,int Plane)
{
   const EpdRect *pRect;
   const EpdText *pText;
   const uint8_t *pGlyph;
   const char *p;
   int i;
   int k;
   int X;
   int GlyphRow;
   uint8_t c;

   for(i = 0; i < gLayer->Rects; i++) {
      pRect = &gLayer->pRects[i];
      if(Row >= pRect->Y && Row < pRect->Y + pRect->H) {
         LayerPixels(pRow,pRect->X,pRect->W,Plane,pRect->Ink);
      }
   }
   for(i = 0; i < gLayer->Texts; i++) {
      pText = &gLayer->pTexts[i];
      if(Row < pText->Y || Row >= pText->Y + 8 * pText->Scale) {
         continue;
      }
      GlyphRow = (Row - pText->Y) / pText->Scale;
      X = pText->X;
      for(p = pText->pText; *p != 0 && X < EPD_WIDTH; p++) {
         c = (uint8_t) *p;
         if(c < 32 || c > 127) {
            c = '?';
         }
         pGlyph = &gFont8x8[(c - 32) * 7];
      // column 0 of the 8x8 cell is blank
         for(k = 0; k < 7; k++) {
            if(pGlyph[k] & (1 << GlyphRow)) {
               LayerPixels(pRow,X + (k + 1) * pText->Scale,pText->Scale,
                           Plane,pText->Ink);
            }
         }
         X += 8 * pText->Scale;
      }
   }
}

// Send one plane from SPI flash. Invert: XOR mask applied to every byte.
// bFill: send Fill instead of reading flash. The layer, if any, goes on top.
static bool EpdSendPlane(uint8_t Cmd,uint32_t Adr,uint8_t Invert,
                         bool bFill,uint8_t Fill)
{
   uint8_t Buf[CHUNK_SIZE];
   uint32_t Done;
   uint32_t Len;
   uint32_t i;
   int Plane = Cmd == 0x10 ? 0 : 1;
   uint32_t Start = ClockP_getSystemTicks();
   bool bRet = false;

   EpdCsLow();
   EpdWrite9(Cmd);
   for(Done = 0; Done < EPD_PLANE_SIZE; Done += Len) {
      Len = EPD_PLANE_SIZE - Done;
      if(Len > sizeof(Buf)) {
         Len = sizeof(Buf);
      }
      if(bFill) {
         memset(Buf,Fill,Len);
      }
      else if(eepromRead(Adr + Done,Buf,Len)) {
         ELOG("flash read failed @ 0x%x\n",Adr + Done);
         bRet = true;
         break;
      }
      for(i = 0; i < Len; i++) {
         Buf[i] ^= Invert;
      }
      if(gLayer != NULL) {
      // CHUNK_SIZE and EPD_PLANE_SIZE are whole rows
         for(i = 0; i < Len; i += EPD_ROW_BYTES) {
            LayerRow(&Buf[i],(Done + i) / EPD_ROW_BYTES,Plane);
         }
      }
      for(i = 0; i < Len; i++) {
         EpdWrite9(0x100 | Buf[i]);
      }
   }
   EpdCsHigh();
   gEpdDiag.PlaneMs[Plane] =
      (ClockP_getSystemTicks() - Start) * ClockP_tickPeriod / 1000;
   return bRet;
}

static uint32_t gPlaneAdr;
static uint8_t gFlags;

static bool EpdSendBw(void)
{
   if(gPlaneAdr == EPD_NO_IMAGE) {
      return EpdSendPlane(0x10,0,0,true,0xff);   // 1 = white
   }
   return EpdSendPlane(0x10,gPlaneAdr,
                       (gFlags & EPD_INVERT_BW) ? 0xff : 0,false,0);
}

static bool EpdSendYellow(void)
{
   if((gFlags & EPD_NO_YELLOW) || gPlaneAdr == EPD_NO_IMAGE) {
      return EpdSendPlane(0x13,0,0,true,0xff);   // 1 = no yellow
   }
   return EpdSendPlane(0x13,gPlaneAdr + EPD_PLANE_SIZE,
                       (gFlags & EPD_INVERT_Y) ? 0xff : 0,false,0);
}

// Record 7: resolution, then the plane(s) selected by record bytes 1 and 2
static bool EpdImage(const uint8_t *pRec)
{
   const uint8_t Tres[] = {
      EPD_WIDTH >> 8, EPD_WIDTH & 0xff, EPD_HEIGHT >> 8, EPD_HEIGHT & 0xff
   };
   bool bRet = false;

   EpdCmd(0x61,Tres,sizeof(Tres));
   // No VDCS (0x82): stock only sends it when the NVRAM holds a VCOM value,
   // and this tag's NVRAM has none.
   if(pRec[2] == 0) {
      bRet = EpdSendBw();
      if(!bRet) {
         bRet = EpdSendYellow();
      }
   }
   else if(pRec[2] >= 2) {
      if(pRec[1] == 2 || pRec[1] == 5) {
         bRet = EpdSendBw();
      }
      else {
         bRet = EpdSendYellow();
      }
   }
   return bRet;
}

static bool EpdRunScript(const uint8_t *p,const uint8_t *pEnd)
{
   uint16_t Arg;
   bool bRet = false;

   while(!bRet) {
      if(p + 3 > pEnd) {
         ELOG("script overrun\n");
         return true;
      }
      Arg = p[1] | (p[2] << 8);
      EpdTrace(TR_RECORD | (p[0] & 0x0f));
      switch(p[0]) {
         case 0:     // end
            return false;

         case 1:     // command + (len - 1) data bytes
            if(Arg == 0 || p + 3 + Arg > pEnd) {
               ELOG("bad command record\n");
               return true;
            }
#ifdef EPD_BTST
         // Experiment: booster soft start (06) with our own 3 bytes, e.g.
         // EPD_BTST=0x572B17, instead of the script's 57 2B 38
            if(p[3] == 0x06 && Arg == 4) {
               const uint8_t Btst[3] = {
                  (EPD_BTST >> 16) & 0xff, (EPD_BTST >> 8) & 0xff, EPD_BTST & 0xff
               };

               ALOG("BTST %02x %02x %02x\n",Btst[0],Btst[1],Btst[2]);
               EpdCmd(0x06,Btst,sizeof(Btst));
               p += 3 + Arg;
               continue;
            }
#endif
            EpdCmd(p[3],&p[4],Arg - 1);
            p += 3 + Arg;
            continue;

         case 2:     // delay in ms
            EpdSleep(Arg);
            break;

         case 3:     // wait while busy
         case 5:     // (5 follows the refresh command)
            bRet = EpdWaitBusy(p[0]);
            break;

         case 4: {   // command followed by (len - 1) zero bytes
            const uint8_t Zero = 0;
            int i;

            EpdCsLow();
            EpdWrite9(p[3]);
            for(i = 1; i < Arg; i++) {
               EpdWrite9(0x100 | Zero);
            }
            EpdCsHigh();
            p += 4;
            continue;
         }

         case 6:     // reset + hidden registers
            EpdReset();
            break;

         case 7:     // image
            bRet = EpdImage(p);
            break;

         case 9:     // panel power, byte 1 bit 0 = on
            EpdPower(p[1] & 1);
            break;

         default:    // 8 (status read), 0xa, 0xb: not used by this script
            ELOG("unsupported record 0x%x\n",p[0]);
            return true;
      }
      p += 3;
   }
   return bRet;
}

static uint32_t ScriptU32(const uint8_t *p)
{
   return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

// Run segment 1, the temperature band's segment 2 and segment 3
static bool EpdRunSegments(const uint8_t *pScript)
{
   const uint8_t *pBody = &pScript[SCRIPT_BODY];
   const uint8_t *pEnd = &pScript[SCRIPT_MAX_LEN];
   uint32_t Max = SCRIPT_MAX_LEN - SCRIPT_BODY;
   uint32_t Seg1 = ScriptU32(&pBody[0]);
   uint32_t Bands = ScriptU32(&pBody[4]);
   uint32_t Seg3 = ScriptU32(&pBody[8]);
   uint32_t Seg2 = 0xffffffff;
   uint32_t Band;
   int Temp = EPD_TEMPERATURE;
   int8_t Threshold;

   for(Band = Bands; Band + SCRIPT_BAND_SIZE <= Max; Band += SCRIPT_BAND_SIZE) {
      Threshold = (int8_t) pBody[Band];
      if(Threshold < SCRIPT_BAND_END) {
         break;
      }
   // the coldest band if none matches (stock doesn't draw then)
      Seg2 = ScriptU32(&pBody[Band + 1]);
      if(Threshold <= Temp) {
         break;
      }
   }
   if(Seg1 >= Max || Seg2 >= Max || Seg3 >= Max) {
      ELOG("script body doesn't parse, running 0x%x..END\n",SCRIPT_RECORDS);
      return EpdRunScript(&pScript[SCRIPT_RECORDS],pEnd);
   }
   ALOG("panel script: %d C, segments 0x%x 0x%x 0x%x\n",Temp,Seg1,Seg2,Seg3);
   return EpdRunScript(&pBody[Seg1],pEnd)
          || EpdRunScript(&pBody[Seg2],pEnd)
          || EpdRunScript(&pBody[Seg3],pEnd);
}

bool EpdDrawFromFlash(uint32_t PlaneAdr,uint8_t Flags)
{
   static uint8_t pScript[SCRIPT_MAX_LEN];   // the heap is only 256 bytes
   uint32_t Magic;
   bool bRet = true;

   gPlaneAdr = PlaneAdr;
   gFlags = Flags;
   memset(&gEpdDiag,0,sizeof(gEpdDiag));
   gEpdDiag.Magic = 0xEDD1A600;
   EpdTrace(TR_DRAW_START);
   do {
      if(eepromRead(EEPROM_PANEL_SCRIPT,pScript,SCRIPT_MAX_LEN)) {
         ELOG("script read failed\n");
         break;
      }
      memcpy(&Magic,pScript,sizeof(Magic));
      if(Magic != SCRIPT_MAGIC || pScript[4] != 4 || pScript[5] != 3) {
         ELOG("no panel script (magic 0x%x, format %d %d)\n",Magic,
              pScript[4],pScript[5]);
         break;
      }
      bRet = EpdRunSegments(pScript);
   } while(false);

   EpdPower(false);
   EpdTrace(TR_DRAW_DONE);
   gEpdDiag.Result = bRet ? 2 : 1;
   if(gEpdDiag.Samples != 0) {
      gEpdRefreshMinMv = gEpdDiag.MinMv;
      LOG("VDDS during draw: %u..%u mV, %u samples\n",gEpdDiag.MinMv,
          gEpdDiag.MaxMv,gEpdDiag.Samples);
   }
   return bRet;
}

// Draw an image the OEPL protocol code stored in SPI flash: an
// EepromImageHeader, then the planes (1 = black, 1 = yellow).
void drawImageAtAddress(uint32_t addr, uint8_t lut)
{
   struct EepromImageHeader Hdr;
   uint8_t Flags = EPD_INVERT_BW | EPD_INVERT_Y;

   do {
      if(eepromRead(addr,&Hdr,sizeof(Hdr))) {
         break;
      }
      if(Hdr.validMarker != EEPROM_IMG_VALID) {
         ELOG("image header not valid\n");
         break;
      }
      if(Hdr.dataType == DATATYPE_IMG_RAW_1BPP) {
         Flags |= EPD_NO_YELLOW;
      }
      else if(Hdr.dataType != DATATYPE_IMG_RAW_2BPP) {
      // G5 and zlib images were decoded to raw planes when they arrived
         ELOG("dataType 0x%x not supported\n",Hdr.dataType);
         break;
      }
      ALOG("drawing image, dataType 0x%x\n",Hdr.dataType);
      EpdDrawFromFlash(addr + sizeof(Hdr),Flags);
   } while(false);
}

#ifdef EPD_TEST
// The stock screen slots are still in SPI flash: draw slot 0 (0x15000,
// 0x20 byte header then the two planes).
#ifndef EPD_TEST_SLOT
#define EPD_TEST_SLOT   0x15000
#endif

void EpdTest()
{
   bool bErr;

   ALOG("EPD test: drawing the image at 0x%x\n",EPD_TEST_SLOT);
   bErr = EpdDrawFromFlash(EPD_TEST_SLOT + 0x20,0);
   eepromPower(false);
   ALOG("EPD test %s\n",bErr ? "FAILED" : "done");
   while(true) {
      WaitMs(1000);
   }
}
#endif   // EPD_TEST
#endif   // CHROMA_74H
