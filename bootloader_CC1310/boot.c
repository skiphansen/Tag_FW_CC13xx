/*
 * Chroma 74H bootloader, flash sector 0 of the CC1310F64.
 *
 * Applies an OTA image the OEPL app staged in SPI flash, then starts the
 * app at OTA_APP_BASE. See ../oepl_CC1310/ota_74h.h for the layout and
 * the record format. It never writes sector 0 or the CCFG sector, so a
 * reset or power cut at any point leaves either a working app or an armed
 * record the next boot applies again.
 *
 * Kept small and self-contained: interrupts stay off, no TI drivers, the
 * SPI flash is bit banged and internal flash is written with driverlib's
 * ROM-backed FlashSectorErase()/FlashProgram().
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_nvic.h)
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/gpio.h)
#include DeviceFamily_constructPath(driverlib/flash.h)
#include DeviceFamily_constructPath(driverlib/vims.h)
#include DeviceFamily_constructPath(driverlib/setup.h)
#include "ota_74h.h"

// SPI flash pins (Chroma_CC1310.h CHROMA_SPI0_* / CHROMA_SPI_FLASH_CS)
#define FLASH_MOSI         IOID_8
#define FLASH_CLK          IOID_9
#define FLASH_MISO         IOID_10
#define FLASH_CS           IOID_11

#define CMD_PP             0x02
#define CMD_READ           0x03
#define CMD_RDSR           0x05
#define CMD_WREN           0x06
#define CMD_RDP            0xab
#define CMD_DP             0xb9

#define APPLY_TRIES        5

static uint8_t gBuf[OTA_SECTOR_SIZE];

static void SpiOpen(void)
{
   PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
   while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
   PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
   PRCMLoadSet();
   while(!PRCMLoadGet());

   GPIO_setDio(FLASH_CS);
   GPIO_clearDio(FLASH_CLK);
   IOCPortConfigureSet(FLASH_CS,IOC_PORT_GPIO,IOC_STD_OUTPUT);
   IOCPortConfigureSet(FLASH_CLK,IOC_PORT_GPIO,IOC_STD_OUTPUT);
   IOCPortConfigureSet(FLASH_MOSI,IOC_PORT_GPIO,IOC_STD_OUTPUT);
   IOCPortConfigureSet(FLASH_MISO,IOC_PORT_GPIO,
                       (IOC_STD_INPUT & ~IOC_NO_IOPULL) | IOC_IOPULL_UP);
   GPIO_setOutputEnableDio(FLASH_CS,GPIO_OUTPUT_ENABLE);
   GPIO_setOutputEnableDio(FLASH_CLK,GPIO_OUTPUT_ENABLE);
   GPIO_setOutputEnableDio(FLASH_MOSI,GPIO_OUTPUT_ENABLE);
}

// Mode 0, MSB first
static uint8_t SpiByte(uint8_t Out)
{
   uint8_t In = 0;
   int i;

   for(i = 0; i < 8; i++) {
      GPIO_writeDio(FLASH_MOSI,(Out & 0x80) ? 1 : 0);
      Out <<= 1;
      GPIO_setDio(FLASH_CLK);
      In = (In << 1) | GPIO_readDio(FLASH_MISO);
      GPIO_clearDio(FLASH_CLK);
   }
   return In;
}

static void SpiCmdAdr(uint8_t Cmd,uint32_t Adr)
{
   GPIO_clearDio(FLASH_CS);
   SpiByte(Cmd);
   SpiByte((uint8_t)(Adr >> 16));
   SpiByte((uint8_t)(Adr >> 8));
   SpiByte((uint8_t) Adr);
}

static void SpiSimpleCmd(uint8_t Cmd)
{
   GPIO_clearDio(FLASH_CS);
   SpiByte(Cmd);
   GPIO_setDio(FLASH_CS);
}

static void SpiRead(uint32_t Adr,void *pDst,uint32_t Len)
{
   uint8_t *p = (uint8_t *) pDst;

   SpiCmdAdr(CMD_READ,Adr);
   while(Len--) {
      *p++ = SpiByte(0);
   }
   GPIO_setDio(FLASH_CS);
}

static void SpiWaitIdle(void)
{
   uint32_t Tries = 1000000;

   GPIO_clearDio(FLASH_CS);
   SpiByte(CMD_RDSR);
   while((SpiByte(0) & 1) && --Tries);
   GPIO_setDio(FLASH_CS);
}

// Program one byte (bits 1 -> 0 only)
static void SpiProgramByte(uint32_t Adr,uint8_t Data)
{
   SpiSimpleCmd(CMD_WREN);
   SpiCmdAdr(CMD_PP,Adr);
   SpiByte(Data);
   GPIO_setDio(FLASH_CS);
   SpiWaitIdle();
}

static uint32_t StagedCrc(const OtaRecord *pRec)
{
   uint32_t Crc = 0;
   uint32_t Done;
   uint32_t Len;

   for(Done = 0; Done < pRec->ImageLen; Done += Len) {
      Len = pRec->ImageLen - Done;
      if(Len > sizeof(gBuf)) {
         Len = sizeof(gBuf);
      }
      SpiRead(pRec->ImageAdr + Done,gBuf,Len);
      Crc = OtaCrc32(Crc,gBuf,Len);
   }
   return Crc;
}

// Copy the staged image to OTA_APP_BASE, sector by sector with read back.
// Returns true on success.
static bool Apply(const OtaRecord *pRec)
{
   uint32_t Done;
   uint32_t Len;
   int Try;

   for(Done = 0; Done < pRec->ImageLen; Done += Len) {
      Len = pRec->ImageLen - Done;
      if(Len > sizeof(gBuf)) {
         Len = sizeof(gBuf);
      }
      SpiRead(pRec->ImageAdr + Done,gBuf,Len);
      for(Try = 0; Try < APPLY_TRIES; Try++) {
         if(FlashSectorErase(OTA_APP_BASE + Done) == FAPI_STATUS_SUCCESS
            && FlashProgram(gBuf,OTA_APP_BASE + Done,Len) == FAPI_STATUS_SUCCESS
            && memcmp((void *)(OTA_APP_BASE + Done),gBuf,Len) == 0)
         {
            break;
         }
      }
      if(Try == APPLY_TRIES) {
         return false;
      }
   }
   return OtaCrc32(0,(const uint8_t *) OTA_APP_BASE,pRec->ImageLen) == pRec->ImageCrc;
}

static void ApplyIfArmed(void)
{
   OtaRecord Rec;
   bool bApplied;

   SpiRead(OTA_RECORD_ADR,&Rec,sizeof(Rec));
   if(Rec.Magic != OTA_RECORD_MAGIC || Rec.Armed != 0x00 || Rec.Done != 0xff) {
      return;
   }
   if(Rec.ImageLen < 8 || Rec.ImageLen > OTA_APP_MAX
      || StagedCrc(&Rec) != Rec.ImageCrc)
   {
   // A bad staged image: drop the record, keep the current app
      SpiProgramByte(OTA_RECORD_ADR + OTA_DONE_OFFSET,0x00);
      return;
   }
// Flash writes: no cache or line buffer (they would serve stale data)
   VIMSModeSafeSet(VIMS_BASE,VIMS_MODE_DISABLED,true);
   VIMSLineBufDisable(VIMS_BASE);
   bApplied = Apply(&Rec);
   VIMSLineBufEnable(VIMS_BASE);
   VIMSModeSafeSet(VIMS_BASE,VIMS_MODE_ENABLED,true);
   if(bApplied) {
      SpiProgramByte(OTA_RECORD_ADR + OTA_DONE_OFFSET,0x00);
   }
// else: the record stays armed and the next boot tries again
}

static bool AppValid(void)
{
   uint32_t Sp = HWREG(OTA_APP_BASE);
   uint32_t Pc = HWREG(OTA_APP_BASE + 4);

   return Sp > SRAM_BASE && Sp <= SRAM_BASE + 0x4000
          && (Pc & 1) != 0
          && Pc > OTA_APP_BASE && Pc < OTA_APP_BASE + OTA_APP_MAX;
}

// Deep power-down, as stock leaves it; the app wakes it with CMD_RDP. CS
// stays driven high (and so the GPIO module and its power domain stay on)
// until the app's PIN init takes the pins over.
static void SpiClose(void)
{
   SpiSimpleCmd(CMD_DP);
}

static void JumpToApp(void)
{
   HWREG(NVIC_VTABLE) = OTA_APP_BASE;
   CPUcpsie();
   __asm("    movw r0, #0x1000");
   __asm("    ldr  r1, [r0]");
   __asm("    msr  msp, r1");
   __asm("    ldr  r0, [r0, #4]");
   __asm("    bx   r0");
}

int main(void)
{
   CPUcpsid();
   SpiOpen();
   SpiSimpleCmd(CMD_RDP);
   CPUdelay(16000);           // > 35 us to leave deep power-down (~1 ms)
   SpiWaitIdle();
   ApplyIfArmed();
   SpiClose();
   if(AppValid()) {
      JumpToApp();
   }
// Nothing to run: wait for JTAG
   while(true) {
   }
}

// ---- startup ----------------------------------------------------------

extern void _c_int00(void);
extern uint32_t __STACK_END;

void ResetISR(void)
{
   SetupTrimDevice();
   _c_int00();
}

static void FaultISR(void)
{
   while(true) {
   }
}

#pragma RETAIN(gVectors)
#pragma DATA_SECTION(gVectors, ".intvecs")
void (* const gVectors[16])(void) = {
   (void (*)(void))((uint32_t) &__STACK_END),
   ResetISR,      // reset
   FaultISR,      // NMI
   FaultISR,      // hard fault
   FaultISR,      // MPU fault
   FaultISR,      // bus fault
   FaultISR,      // usage fault
   0,0,0,0,
   FaultISR,      // SVCall
   FaultISR,      // debug monitor
   0,
   FaultISR,      // PendSV
   FaultISR       // SysTick
};
