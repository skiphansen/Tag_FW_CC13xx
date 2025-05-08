#include <ti/drivers/SPI.h>

#ifdef DeviceFamily_CC13X0
#include <ti/drivers/pin/PINCC26XX.h>
#else
#include <ti/drivers/GPIO.h>
#endif

#include "eeprom.h"
#include "board.h"
#include "logging.h"

#define CMD_PP          0x02
#define CMD_READ        0x03
#define CMD_RDSR        0x05
#define CMD_WREN        0x06
#define CMD_ERASE_4K    0x20
#define CMD_RDSFDP      0x5a
#define CMD_REMS        0x90
#define CMD_RDP         0xab
#define CMD_DP          0xb9

#define SPI_FLASH_CS    IOID_11
#define FLASH_SPI       0

SPI_Handle gSpi = NULL;

#ifdef DeviceFamily_CC13X1
#include "ti_drivers_config.h"
#define SPI_MODE              SPI_CONTROLLER
#define eepromPrvSelect()     GPIO_write(CONFIG_GPIO_FLASH_CS,0)
#define eepromPrvDeselect()   GPIO_write(CONFIG_GPIO_FLASH_CS,1)
#else
PIN_State gPinState;
PIN_Handle gPinHandle;
#define SPI_MODE              SPI_MASTER
#define eepromPrvSelect()     PIN_setOutputValue(gPinHandle,SPI_FLASH_CS,0)
#define eepromPrvDeselect()   PIN_setOutputValue(gPinHandle,SPI_FLASH_CS,1)
#endif

static bool eepromPrvSimpleCmd(uint8_t Cmd) 
{
   SPI_Transaction transaction;
   bool bRet = false;

   transaction.count = sizeof(Cmd);
   transaction.txBuf = (void *)&Cmd;
   transaction.rxBuf = NULL;

   eepromPrvSelect();
   if(!SPI_transfer(gSpi,&transaction)) {
      ELOG("SPI_transfer failed\n");
      bRet = true;
   }
   eepromPrvDeselect();
   return bRet;
}

static bool eepromPrvCmdAddr(uint8_t Opcode,uint32_t addr) 
{
   SPI_Transaction transaction;
   bool bRet = false;
   uint8_t Cmd[5];

   Cmd[0] = Opcode;
   Cmd[1] = (uint8_t) (addr >> 16);
   Cmd[2] = (uint8_t) (addr >> 8);
   Cmd[3] = (uint8_t) (addr & 0xff);
   Cmd[4] = 0;    // dummy (RDSFDP only)

   transaction.count = Opcode == CMD_RDSFDP ? 5 : 4;
   transaction.txBuf = (void *)&Cmd;
   transaction.rxBuf = NULL;

   eepromPrvSelect();
   if(!SPI_transfer(gSpi,&transaction)) {
      ELOG("SPI_transfer failed\n");
      bRet = true;
   }
   return bRet;
}

bool eepromInit()
{
   SPI_Params spiParams;
   bool bRet = true;

   SPI_Params_init(&spiParams);
   spiParams.bitRate      = 12000000;  // 12 Mhz is the maximum for CC13XX
   spiParams.mode         = SPI_MODE;
   spiParams.transferMode = SPI_MODE_BLOCKING;
   spiParams.frameFormat = SPI_POL0_PHA0;

   do {
      SPI_init();
#ifdef DeviceFamily_CC13X0
      PIN_Config SpiPins[] = {
          /* SPI Flash CS */
         IOID_11 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL |
                  PIN_INPUT_DIS | PIN_DRVSTR_MED,
          PIN_TERMINATE
      };
      gPinHandle = PIN_open(&gPinState,SpiPins);
      if(gPinHandle == NULL) {
         ELOG("PIN_open failed\n");
         break;
      }
#else
      int_fast16_t Err;
      Err = GPIO_setConfig(CONFIG_GPIO_FLASH_CS,
                           GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
      if(Err != GPIO_STATUS_SUCCESS) {
         LOG("GPIO_setConfig failed %d\n",Err);
         break;
      }
#endif

      if((gSpi = SPI_open(FLASH_SPI,&spiParams)) == NULL) {
         ELOG("SPI_open failed\n");
         break;
      }
      bRet = false;
   } while(false);

   return bRet;
}



uint32_t eepromGetSize(void)
{
   return EEPROM_SIZE;
}

bool eepromReadInternal(uint8_t Cmd,uint32_t addr,void *pDst,uint32_t len)
{
   SPI_Transaction transaction;
   bool bRet = true; // assume the worse

   do {
      if((addr + len) > EEPROM_SIZE) {
         ELOG("Invalid addr 0x%x\n",addr);
         break;
      }
      if(gSpi == NULL && eepromPower(true)) {
         break;
      }

      if(eepromPrvCmdAddr(Cmd,addr)) {
         break;
      }
    // Read the data
      transaction.count = len;
      transaction.txBuf = NULL;
      transaction.rxBuf = pDst;
      if(!SPI_transfer(gSpi,&transaction)) {
         ELOG("SPI_transfer failed\n");
         break;
      }
      bRet = false;
      VDUMP_HEX(pDst,len);
   } while(false);
   eepromPrvDeselect();

   return bRet;
}

bool eepromRead(uint32_t addr,void *pDst,uint32_t len)
{
   LOG("read 0x%x Len: %d\n", addr, len);
   return eepromReadInternal(CMD_READ,addr,pDst,len);
}

bool eepromGetSFDP(void *pDst,uint32_t len)
{
   return eepromReadInternal(CMD_RDSFDP,0,pDst,len);
}

bool eepromGetID(void *pDst)
{
   return eepromReadInternal(CMD_REMS,0,pDst,2);
}


// Wait for any write operation to complete
static void eepromPrvBusyWait(void) 
{
   SPI_Transaction transaction;
   uint8_t Cmd = CMD_RDSR;
   uint8_t Status;

   do {
      transaction.count = sizeof(Cmd);
      transaction.txBuf = &Cmd;
      transaction.rxBuf = NULL;
      eepromPrvSelect();
      if(!SPI_transfer(gSpi,&transaction)) {
         ELOG("SPI_transfer failed\n");
         break;
      }
      transaction.txBuf = NULL;
      transaction.rxBuf = &Status;
      while(true) {
         if(!SPI_transfer(gSpi,&transaction)) {
            ELOG("SPI_transfer failed\n");
            break;
         }
         VLOG("Status 0x%x\n",Status);
         if((Status & 1) == 0) {
            break;
         }
      }
   } while(false);
   eepromPrvDeselect();
}

bool eepromWrite(uint32_t addr,void *pSrc,uint32_t len)
{
   bool bRet = false; // assume the best
   SPI_Transaction transaction;
   uint16_t lenNow;
   uint8_t *pU8 = (uint8_t *) pSrc;

   LOG("write 0x%x Len: %d\n", addr, len);
   VDUMP_HEX(pSrc,len);
   while(len) {
      if(gSpi == NULL && eepromPower(true)) {
         break;
      }
      lenNow = EEPROM_WRITE_PAGE_SZ - (addr & (EEPROM_WRITE_PAGE_SZ - 1));

      if(lenNow > len) {
         lenNow = len;
      }
      eepromPrvBusyWait();

      if(eepromPrvSimpleCmd(CMD_WREN)) {
         break;
      }

      if(eepromPrvCmdAddr(CMD_PP,addr)) {
         break;
      }


      LOG("write Len: %d @ 0x%x\n",lenNow,addr);

      transaction.count = lenNow;
      transaction.rxBuf = NULL;
      transaction.txBuf = pU8;
      if(!SPI_transfer(gSpi,&transaction)) {
         ELOG("SPI_transfer failed\n");
         break;
      }

      eepromPrvDeselect();
      addr += lenNow;
      pU8 += lenNow;
      len -= lenNow;
   }
   eepromPrvDeselect();

   if(len != 0) {
      bRet = true;
   }

   return bRet;
}

bool eepromErase(uint32_t addr,uint32_t len)
{
   int NumSectors = len / EEPROM_ERZ_SECTOR_SZ;
   bool bRet = false; // assume the best
   uint8_t now;
   uint8_t Cmd;
   uint32_t SavedAddr = addr;

   LOG("erase 0x%x Len: %d\n",addr,len);
   do {
      if(gSpi == NULL && eepromPower(true)) {
         break;
      }
      if((addr % EEPROM_ERZ_SECTOR_SZ) != 0 ||
         addr > (EEPROM_SIZE - EEPROM_ERZ_SECTOR_SZ)) {
         ELOG("Invalid addr 0x%x\n",addr);
         break;
      }

      if(len % EEPROM_ERZ_SECTOR_SZ) {
         NumSectors++;
      }

      while(NumSectors > 0) {

         if(eepromPrvSimpleCmd(CMD_WREN)) {
            break;
         }
#ifdef CMD_ERASE_64K
         if(nSec >= 16 && !(uint16_t)addr && mOpcodeErz64K) {
         // erase 64K
            Cmd = CMD_ERASE_64K;
            now = 16;
         }
         else
#endif
#ifdef CMD_ERASE_32K
            if(nSec >= 8 && !(((uint16_t)addr) & 0x7fff) && mOpcodeErz32K) {
         // erase 32K
            Cmd = CMD_ERASE_32K;
            now = 8;
         }
         else
#endif
         {
         // erase 4K
            now = 1;
            Cmd = CMD_ERASE_4K;
         }
         VLOG("Erase %d sectors @ 0x%x\n",now,addr);

         if(eepromPrvCmdAddr(Cmd,addr)) {
            break;
         }
         eepromPrvDeselect();
         eepromPrvBusyWait();
         addr += now * EEPROM_ERZ_SECTOR_SZ;
         NumSectors -= now;
      }
   } while(false);

   if(NumSectors != 0) {
      eepromPrvDeselect();
      bRet = true;
   }

   return bRet;
}

bool eepromPower(bool bPowerUp)
{
   bool bRet = false;

   if(!bPowerUp && gSpi != NULL) {
      eepromPrvBusyWait();

      eepromPrvSimpleCmd(CMD_DP);
      SPI_close(gSpi);
      gSpi = NULL;
   }
   else if(gSpi == NULL){
      if(eepromInit()) {
         bRet = true;
      }
      else {
         eepromPrvSimpleCmd(CMD_RDP);
      }
   }

   return bRet;
}

