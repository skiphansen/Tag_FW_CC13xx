#include <stdint.h>
#include <string.h>

#include "main.h"
#include "syncedproto.h"
#include "oepl-proto.h"
#include "oepl-definitions.h"
#include "powermgt.h"
#include "eeprom.h"
#include "drawing.h"
#include "oepl_board.h"
#include "radio.h"
#ifdef G5_SUPPORT
#include "g5.h"
#endif
#ifdef ZLIB_SUPPORT
#include "zlib_decode.h"
#endif

// #define DEBUGBLOCKS

#ifdef DEBUGBLOCKS
#define VERBOSE_DEUG_LOGGING
#endif

#include "logging.h"

// download-stuff
uint8_t blockXferBuffer[BLOCK_XFER_BUFFER_SIZE];
struct blockRequest curBlock = {0};     // used by the block-requester, contains the next request that we'll send
struct AvailDataInfo curDataInfo = {0}; // last 'AvailDataInfo' we received from the AP
bool requestPartialBlock = false;       // if we should ask the AP to get this block from the host or not
#define BLOCK_TRANSFER_ATTEMPTS 5

uint8_t prevImgSlot = 0xFF;
uint8_t curImgSlot = 0xFF;            // the current regular image
static uint8_t shownImgSlot = 0xFF;   // slot last drawn from SPI flash
static uint8_t xferImgSlot = 0xFF;    // slot the running download goes to

// AvailDataInfo.dataTypeArgument (struct imageDataTypeArgStruct, bit fields
// from bit 0: lut:2, preloadImage:1, specialType:5)
#define ARG_PRELOAD        0x04
#define ARG_SPECIAL(a)     ((uint8_t)(a) >> 3)
uint32_t curHighSlotId = 0;
uint8_t nextImgSlot = 0;
#define imgSlots IMAGE_SLOTS;
uint8_t drawWithLut = 0;

// stuff we need to keep track of related to the network/AP
uint8_t APmac[8] = {0};
uint16_t APsrcPan = 0;
uint8_t mSelfMac[8] = {0};

uint8_t seq = 0;
uint8_t currentChannel = 0;

// buffer we use to prepare packets
static uint8_t outBuffer[128] = {0};

// tools
static uint8_t getPacketType(const void *buffer)
{
   const struct MacFcs *fcs = buffer;
   uint8_t type = 0;

   if((fcs->frameType == 1) && (fcs->destAddrType == 2) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 0)) {
   // broadcast frame
      type = ((uint8_t *)buffer)[sizeof(struct MacFrameBcast)];
   }
   else if((fcs->frameType == 1) && (fcs->destAddrType == 3) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 1)) {
   // normal frame
      type = ((uint8_t *)buffer)[sizeof(struct MacFrameNormal)];
   }
   return type;
}

static bool pktIsUnicast(const void *buffer)
{
   const struct MacFcs *fcs = buffer;
   if((fcs->frameType == 1) && (fcs->destAddrType == 2) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 0)) {
      return false;
   }
   else if((fcs->frameType == 1) && (fcs->destAddrType == 3) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 1)) {
   // normal frame
      return true;
   }
    // unknown type...
   return false;
}

static bool checkCRC(const void *p, const uint8_t len)
{
   uint8_t total = 0;
   for(uint8_t c = 1; c < len; c++) {
      total += ((uint8_t *)p)[c];
   }
    // LOG("CRC: rx %d, calc %d\n", ((uint8_t *)p)[0], total);
   return((uint8_t *)p)[0] == total;
}

static void addCRC(void *p, const uint8_t len)
{
   uint8_t total = 0;
   for(uint8_t c = 1; c < len; c++) {
      total += ((uint8_t *)p)[c];
   }
   ((uint8_t *)p)[0] = total;
}

// radio stuff
void sendPing()
{
   struct MacFrameBcast *txframe = (struct MacFrameBcast *)(outBuffer + 1);
   memset(outBuffer, 0, sizeof(struct MacFrameBcast) + 2 + 4);
   outBuffer[0] = sizeof(struct MacFrameBcast) + 1 + 2;
   outBuffer[sizeof(struct MacFrameBcast) + 1] = PKT_PING;
   memcpy(txframe->src, mSelfMac, 8);
   txframe->fcs.frameType = 1;
   txframe->fcs.ackReqd = 1;
   txframe->fcs.destAddrType = 2;
   txframe->fcs.srcAddrType = 3;
   txframe->seq = seq++;
   txframe->dstPan = PROTO_PAN_ID_SUBGHZ;
   txframe->dstAddr = 0xFFFF;
   txframe->srcPan = PROTO_PAN_ID_SUBGHZ;
   commsTxNoCpy(outBuffer);
}

uint8_t detectAP(const uint8_t channel)
{
   uint8_t Ret = 0;
   for(uint8_t c = 1; c <= MAXIMUM_PING_ATTEMPTS; c++) {
      LOG("Send ping %d on channel %d\n",c,channel);
      sendPing();
      uint8_t *inBuffer = commsRxUnencrypted(PING_REPLY_WINDOW);
      if(inBuffer != NULL) {
         if((inBuffer[sizeof(struct MacFrameNormal) + 1] == channel) && (getPacketType(inBuffer) == PKT_PONG)) {
            if(pktIsUnicast(inBuffer)) {
               struct MacFrameNormal *f = (struct MacFrameNormal *)inBuffer;
               memcpy(APmac, f->src, 8);
               APsrcPan = f->pan;
               Ret++;
            }
         }
      }
   }
   return Ret;
}

// data xfer stuff
static void sendShortAvailDataReq()
{
   struct MacFrameBcast *txframe = (struct MacFrameBcast *)(outBuffer + 1);
   outBuffer[0] = sizeof(struct MacFrameBcast) + 1 + 2;
   outBuffer[sizeof(struct MacFrameBcast) + 1] = PKT_AVAIL_DATA_SHORTREQ;
   memcpy(txframe->src, mSelfMac, 8);
   outBuffer[1] = 0x21;
   outBuffer[2] = 0xC8; // quickly set txframe fcs structure for broadcast packet
   txframe->seq = seq++;
   txframe->dstPan = PROTO_PAN_ID_SUBGHZ;
   txframe->dstAddr = 0xFFFF;
   txframe->srcPan = PROTO_PAN_ID_SUBGHZ;
   commsTxNoCpy(outBuffer);
}

static void sendAvailDataReq()
{
   struct MacFrameBcast *txframe = (struct MacFrameBcast *)(outBuffer + 1);
   memset(outBuffer, 0, sizeof(struct MacFrameBcast) + sizeof(struct AvailDataReq) + 2 + 4);
   struct AvailDataReq *availreq = (struct AvailDataReq *)(outBuffer + 2 + sizeof(struct MacFrameBcast));
   outBuffer[0] = sizeof(struct MacFrameBcast) + sizeof(struct AvailDataReq) + 2 + 2;
   outBuffer[sizeof(struct MacFrameBcast) + 1] = PKT_AVAIL_DATA_REQ;
   memcpy(txframe->src, mSelfMac, 8);
   txframe->fcs.frameType = 1;
   txframe->fcs.ackReqd = 1;
   txframe->fcs.destAddrType = 2;
   txframe->fcs.srcAddrType = 3;
   txframe->seq = seq++;
   txframe->dstPan = PROTO_PAN_ID_SUBGHZ;
   txframe->dstAddr = 0xFFFF;
   txframe->srcPan = PROTO_PAN_ID_SUBGHZ;
// TODO: send some (more) meaningful data
   availreq->hwType = HW_TYPE;
   availreq->wakeupReason = wakeUpReason;
   availreq->lastPacketRSSI = mLastRSSI;
   availreq->lastPacketLQI = mLastLqi;
   availreq->temperature = temperature;
   availreq->batteryMv = batteryVoltage;
   availreq->capabilities = capabilities;
   availreq->tagSoftwareVersion = FW_VERSION;
// The AP drops check-ins from unknown MACs whose channel disagrees with its own
   availreq->currentChannel = currentChannel;
   addCRC(availreq, sizeof(struct AvailDataReq));
   commsTxNoCpy(outBuffer);
}

struct AvailDataInfo *getAvailDataInfo() 
{
   radioRxEnable(true);
   for(uint8_t c = 0; c < DATA_REQ_MAX_ATTEMPTS; c++) {
      sendAvailDataReq();
      uint8_t *inBuffer = commsRxUnencrypted(DATA_REQ_RX_WINDOW_SIZE);
      if(inBuffer != NULL) {
         if(getPacketType(inBuffer) == PKT_AVAIL_DATA_INFO) {
            if(checkCRC(inBuffer + sizeof(struct MacFrameNormal) + 1, sizeof(struct AvailDataInfo))) {
               struct MacFrameNormal *f = (struct MacFrameNormal *)inBuffer;
               memcpy(APmac, f->src, 8);
               APsrcPan = f->pan;
               dataReqLastAttempt = c;
               return(struct AvailDataInfo *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
            }
         }
      }
   }
   dataReqLastAttempt = DATA_REQ_MAX_ATTEMPTS;
   return NULL;
}

struct AvailDataInfo *getShortAvailDataInfo() 
{
   radioRxEnable(true);
   for(uint8_t c = 0; c < DATA_REQ_MAX_ATTEMPTS; c++) {
      sendShortAvailDataReq();
      uint8_t *inBuffer = commsRxUnencrypted(DATA_REQ_RX_WINDOW_SIZE);
      if(inBuffer != NULL) {
         if(getPacketType(inBuffer) == PKT_AVAIL_DATA_INFO) {
            if(checkCRC(inBuffer + sizeof(struct MacFrameNormal) + 1, sizeof(struct AvailDataInfo))) {
               struct MacFrameNormal *f = (struct MacFrameNormal *)inBuffer;
               memcpy(APmac, f->src, 8);
               APsrcPan = f->pan;
               dataReqLastAttempt = c;
               return(struct AvailDataInfo *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
            }
         }
      }
   }
   dataReqLastAttempt = DATA_REQ_MAX_ATTEMPTS;
   return NULL;
}

static bool processBlockPart(const struct blockPart *bp)
{
   uint16_t start = bp->blockPart * BLOCK_PART_DATA_SIZE;
   uint16_t size = BLOCK_PART_DATA_SIZE;

   bool bRet = false; // Assume the worse

   do {
   // validate if it's okay to copy data
      if(bp->blockId != curBlock.blockId) {
         VLOG("got a packet for block 0x%02X, expected 0x%X\n",
              bp->blockId,curBlock.blockId);
         break;
      }
      if(start >= (sizeof(blockXferBuffer) - 1)) {
         VLOG("0x%x >= 0x%x\n",start,sizeof(blockXferBuffer) - 1);
         break;
      }
      if(bp->blockPart > BLOCK_MAX_PARTS) {
         VLOG("bp->blockPart (%d) > BLOCK_MAX_PARTS\n",bp->blockPart);
         break;
      }
      if((start + size) > sizeof(blockXferBuffer)) {
         VLOG("block len reduced from %d",size);
         size = sizeof(blockXferBuffer) - start;
         VLOG_RAW("to %d\n",size);
      }
      if(!checkCRC(bp, sizeof(struct blockPart) + BLOCK_PART_DATA_SIZE)) {
         ELOG("CRC Failed\n");
         break;
      }
   //  copy block data to buffer
      memcpy((void *)(blockXferBuffer + start),(const void *)bp->data,size);
   // we don't need this block anymore, set bit to 0 so we don't request it again
      curBlock.requestedParts[bp->blockPart / 8] &= ~(1 << (bp->blockPart % 8));
#ifdef VERBOSE_DEUG_LOGGING
      VLOG("Got blockPart %d",bp->blockPart);
      if(bp->blockPart == 0) {
         struct blockData *bd = (struct blockData *) bp->data;
         VLOG_RAW(" size 0x%x checksum 0x%x\n",bd->size,bd->checksum);
      }
      VLOG_RAW("\n");
#endif
      bRet = true;
   } while(false);

   return bRet;
}

static bool blockRxLoop(const uint32_t timeout)
{
   bool success = false;
   uint8_t *inBuffer;
   struct blockPart *bp;
   uint8_t PktType;

   while(true) {
      if((inBuffer = commsRxUnencrypted(timeout)) == NULL) {
         break;
      }
      PktType = getPacketType(inBuffer);
      if(PktType == PKT_BLOCK_PART) {
         bp = (struct blockPart *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
         success = processBlockPart(bp);
      }
      else {
         VLOG("Ignoring PktType 0x%x\n",PktType);
      }
   }
   return success;
}

static struct blockRequestAck *continueToRX(uint8_t *inBuffer) 
{
   struct blockRequestAck *ack = (struct blockRequestAck *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
   ack->pleaseWaitMs = 0;
   return ack;
}

static void sendBlockRequest()
{
   memset(outBuffer, 0, sizeof(struct MacFrameNormal) + sizeof(struct blockRequest) + 2 + 2);
   struct MacFrameNormal *f = (struct MacFrameNormal *)(outBuffer + 1);
   struct blockRequest *blockreq = (struct blockRequest *)(outBuffer + 2 + sizeof(struct MacFrameNormal));
   outBuffer[0] = sizeof(struct MacFrameNormal) + sizeof(struct blockRequest) + 2 + 2;
   if(requestPartialBlock) {
      ;
      outBuffer[sizeof(struct MacFrameNormal) + 1] = PKT_BLOCK_PARTIAL_REQUEST;
   }
   else {
      outBuffer[sizeof(struct MacFrameNormal) + 1] = PKT_BLOCK_REQUEST;
   }
   memcpy(f->src, mSelfMac, 8);
   memcpy(f->dst, APmac, 8);
   f->fcs.frameType = 1;
   f->fcs.secure = 0;
   f->fcs.framePending = 0;
   f->fcs.ackReqd = 0;
   f->fcs.panIdCompressed = 1;
   f->fcs.destAddrType = 3;
   f->fcs.frameVer = 0;
   f->fcs.srcAddrType = 3;
   f->seq = seq++;
   f->pan = APsrcPan;
   memcpy(blockreq, &curBlock, sizeof(struct blockRequest));
    // LOG("req ver: %02X%02X%02X%02X%02X%02X%02X%02X\n", ((uint8_t*)&blockreq->ver)[0],((uint8_t*)&blockreq->ver)[1],((uint8_t*)&blockreq->ver)[2],((uint8_t*)&blockreq->ver)[3],((uint8_t*)&blockreq->ver)[4],((uint8_t*)&blockreq->ver)[5],((uint8_t*)&blockreq->ver)[6],((uint8_t*)&blockreq->ver)[7]);
   addCRC(blockreq, sizeof(struct blockRequest));
   commsTxNoCpy(outBuffer);
}

static struct blockRequestAck *performBlockRequest() 
{
   uint8_t *inBuffer = NULL;
   struct blockRequestAck *pRet = NULL;

   for(uint8_t c = 0; c < 30; c++) {
      sendBlockRequest();
      inBuffer = commsRxUnencrypted(50);
      if(inBuffer != NULL) {
         uint8_t PktType = getPacketType(inBuffer);
         switch(PktType) {
            case PKT_BLOCK_REQUEST_ACK:
               if(checkCRC((inBuffer + sizeof(struct MacFrameNormal) + 1), sizeof(struct blockRequestAck))) {
                  VLOG("Got PKT_BLOCK_REQUEST_ACK\n");
                  pRet = (struct blockRequestAck *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
               }
               else {
                  VLOG("Got PKT_BLOCK_REQUEST_ACK with bad CRC\n");
               }
               break;

            case PKT_BLOCK_PART:
            // block already started while we were waiting for a get block reply
            // LOG("!");
            // processBlockPart((struct blockPart *)(inBuffer + sizeof(struct MacFrameNormal) + 1));
               VLOG("Got PKT_BLOCK_PART while we were waiting for a get block reply\n");
               pRet = continueToRX(inBuffer);
               break;

            case PKT_CANCEL_XFER:
               VLOG("Got PKT_CANCEL_XFER\n");
               break;

            default:
               VLOG("pkt w/type %02X\n", getPacketType(inBuffer));
               break;
         }
         break;
      }
   }
   return pRet;
}

static void sendXferCompletePacket()
{
   memset(outBuffer, 0, sizeof(struct MacFrameNormal) + 2 + 4);
   struct MacFrameNormal *f = (struct MacFrameNormal *)(outBuffer + 1);
   outBuffer[0] = sizeof(struct MacFrameNormal) + 2 + 2;
   outBuffer[sizeof(struct MacFrameNormal) + 1] = PKT_XFER_COMPLETE;
   memcpy(f->src, mSelfMac, 8);
   memcpy(f->dst, APmac, 8);
   f->fcs.frameType = 1;
   f->fcs.secure = 0;
   f->fcs.framePending = 0;
   f->fcs.ackReqd = 0;
   f->fcs.panIdCompressed = 1;
   f->fcs.destAddrType = 3;
   f->fcs.frameVer = 0;
   f->fcs.srcAddrType = 3;
   f->pan = APsrcPan;
   f->seq = seq++;
   VLOG("\n");
   commsTxNoCpy(outBuffer);
}

static void sendXferComplete()
{
   radioRxEnable(true);

   for(uint8_t c = 0; c < 16; c++) {
      sendXferCompletePacket();
      uint8_t *inBuffer = commsRxUnencrypted(130);
      if(inBuffer != NULL) {
         if(getPacketType(inBuffer) == PKT_XFER_COMPLETE_ACK) {
            VLOG("XFC ACK\n");
            return;
         }
      }
   }
   LOG("XFC NACK!\n");
   return;
}

static bool validateBlockData()
{
   struct blockData *bd = (struct blockData *)blockXferBuffer;
   VLOG("expected len = 0x%04X, checksum=0x%04X\n",bd->size,bd->checksum);
   if(bd->size > BLOCK_XFER_BUFFER_SIZE - sizeof(struct blockData)) {
      LOG("Impossible data size, abort!\n");
      return false;
   }
   uint16_t t = 0;
   for(uint16_t c = 0; c < bd->size; c++) {
      t += bd->data[c];
   }
   VLOG("Checked len = 0x%04X, checksum=0x%04X\n", bd->size, t);
   return bd->checksum == t;
}

// EEprom related stuff
static uint32_t getAddressForSlot(const uint8_t s)
{
   return EEPROM_IMG_START + (EEPROM_IMG_EACH * s);
}

static uint8_t findSlot(const uint8_t *ver)
{
    // return 0xFF; // remove me! This forces the tag to re-download each and every upload without checking if it's already in the eeprom somewhere
   uint32_t markerValid = EEPROM_IMG_VALID;
   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
      eepromRead(getAddressForSlot(c),eih,sizeof(struct EepromImageHeader));
      if(!memcmp(&eih->validMarker, &markerValid, 4)) {
         if(!memcmp(&eih->version, (void *)ver, 8)) {
            return c;
         }
      }
   }
   return 0xFF;
}

static void eraseUpdateBlock()
{
   eepromErase(EEPROM_OTA_START,EEPROM_OTA_LEN);
}

static void eraseImageBlock(const uint8_t c)
{
   eepromErase(getAddressForSlot(c), EEPROM_IMG_EACH);
}

// CMD_ERASE_EEPROM_IMAGES: forget every stored image
void eraseImageSlots(void)
{
   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      wdt10s();
      eraseImageBlock(c);
   }
   curImgSlot = 0xFF;
   shownImgSlot = 0xFF;
   nextImgSlot = 0;
   curHighSlotId = 0;
   memset(&curDataInfo,0,sizeof(curDataInfo));
}

static void saveUpdateBlockData(uint8_t blockId)
{
   uint32_t addr = EEPROM_OTA_START+ (blockId * BLOCK_DATA_SIZE);
   void *pSrc = blockXferBuffer + sizeof(struct blockData);

   if(eepromWrite(addr,pSrc,BLOCK_DATA_SIZE)) {
      ELOG("EEPROM write failed\n");
   }
   else {
      VLOG("block 0x%x saved\n",blockId);
   }
}

static void saveImgBlockData(const uint8_t imgSlot, const uint8_t blockId)
{
   uint32_t length = EEPROM_IMG_EACH - (sizeof(struct EepromImageHeader) + (blockId * BLOCK_DATA_SIZE));
   uint32_t addr = getAddressForSlot(imgSlot) + 
                   sizeof(struct EepromImageHeader) + 
                   (blockId * BLOCK_DATA_SIZE);

   if(length > 4096) {
      length = 4096;
   }

   if(eepromWrite(addr,blockXferBuffer + sizeof(struct blockData),length)) {
      LOG("EEPROM write failed\n");
   }
}

void drawImageFromEeprom(const uint8_t imgSlot)
{
   shownImgSlot = imgSlot;
   drawImageAtAddress(getAddressForSlot(imgSlot), drawWithLut);
   drawWithLut = 0; // default back to the regular ol' stock/OTP LUT
}

static uint32_t getHighSlotId()
{
   uint32_t temp = 0;
   uint32_t markerValid = EEPROM_IMG_VALID;
#ifdef OEPL_PRELOADS
   uint32_t HighImageId = 0;
#endif
   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
      eepromRead(getAddressForSlot(c),eih,sizeof(struct EepromImageHeader));
      if(!memcmp(&eih->validMarker, &markerValid, 4)) {
         if(temp < eih->id) {
            temp = eih->id;
            nextImgSlot = c;
         }
#ifdef OEPL_PRELOADS
      // The newest regular image is the one on the panel
         if(!(eih->argument & ARG_PRELOAD) && HighImageId < eih->id) {
            HighImageId = eih->id;
            curImgSlot = c;
         }
#endif
      }
   }
   LOG("found high id=%d in slot %d\n", temp, nextImgSlot);
   return temp;
}

// A slot to download into: not Avoid and not the image on display. With
// OEPL_PRELOADS stored special images (AP lost screen etc.) are kept too,
// unless nothing else is left; then the oldest slot is reused, so a full
// flash (or a header with a stale argument byte) can't block downloads.
static uint8_t findFreeSlot(const uint8_t Avoid)
{
   struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
   uint8_t Slot = nextImgSlot;
   uint8_t Oldest = 0xFF;
   uint32_t OldestId = 0xFFFFFFFF;

   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      if(++Slot >= IMAGE_SLOTS) {
         Slot = 0;
      }
      if(Slot == Avoid || Slot == shownImgSlot) {
         continue;
      }
      eepromRead(getAddressForSlot(Slot),eih,sizeof(struct EepromImageHeader));
      if(eih->validMarker != EEPROM_IMG_VALID) {
         return Slot;
      }
#ifdef OEPL_PRELOADS
      if(ARG_SPECIAL(eih->argument) != CUSTOM_IMAGE_NOCUSTOM) {
         if(eih->id < OldestId) {
            OldestId = eih->id;
            Oldest = Slot;
         }
         continue;
      }
#endif
      return Slot;
   }
   return Oldest;
}

#ifdef OEPL_PRELOADS
// The newest stored image with this special type, 0xFF if none
static uint8_t findSpecialSlot(const uint8_t Type)
{
   struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
   uint8_t Ret = 0xFF;
   uint32_t HighId = 0;

   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      eepromRead(getAddressForSlot(c),eih,sizeof(struct EepromImageHeader));
      if(eih->validMarker == EEPROM_IMG_VALID
         && (eih->argument & ARG_PRELOAD)
         && ARG_SPECIAL(eih->argument) == Type
         && eih->id >= HighId)
      {
         HighId = eih->id;
         Ret = c;
      }
   }
   return Ret;
}

// Only one image of each special type is kept (slideshow images excepted)
static void removeOldSpecial(const uint8_t Argument,const uint8_t Keep)
{
   struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
   uint8_t Type = ARG_SPECIAL(Argument);

   if(!(Argument & ARG_PRELOAD)
      || Type == CUSTOM_IMAGE_NOCUSTOM || Type == CUSTOM_IMAGE_SLIDESHOW)
   {
      return;
   }
   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      if(c == Keep) {
         continue;
      }
      eepromRead(getAddressForSlot(c),eih,sizeof(struct EepromImageHeader));
      if(eih->validMarker == EEPROM_IMG_VALID
         && (eih->argument & ARG_PRELOAD)
         && ARG_SPECIAL(eih->argument) == Type)
      {
         LOG("replacing special image in slot %d\n",c);
         eraseImageBlock(c);
      }
   }
}

// Draw the stored special image of this type. Returns false if there is none.
bool drawSpecialImage(const uint8_t Type)
{
   uint8_t Slot = findSpecialSlot(Type);

   if(Slot == 0xFF) {
      return false;
   }
   LOG("drawing special image %d from slot %d\n",Type,Slot);
   wdt60s();
   drawImageFromEeprom(Slot);
   return true;
}

// Redraw the current regular image (after a special one was shown)
bool drawCurrentImage(void)
{
   if(curImgSlot == 0xFF) {
      return false;
   }
   wdt60s();
   drawImageFromEeprom(curImgSlot);
   return true;
}
#endif

static uint8_t partsThisBlock = 0;
static uint8_t blockAttempts = 0; // these CAN be local to the function, but for some reason, they won't survive sleep?
                                  // they get overwritten with  7F 32 44 20 00 00 00 00 11, I don't know why.

static bool getDataBlock(const uint16_t blockSize)
{
   blockAttempts = BLOCK_TRANSFER_ATTEMPTS;
   if(blockSize == BLOCK_DATA_SIZE) {
      partsThisBlock = BLOCK_MAX_PARTS;
      memset(curBlock.requestedParts,0xFF,BLOCK_REQ_PARTS_BYTES);
   }
   else {
      partsThisBlock = (sizeof(struct blockData) + blockSize) / BLOCK_PART_DATA_SIZE;
      if((sizeof(struct blockData) + blockSize) % BLOCK_PART_DATA_SIZE) {
         partsThisBlock++;
      }
      memset(curBlock.requestedParts, 0x00, BLOCK_REQ_PARTS_BYTES);
      for(uint8_t c = 0; c < partsThisBlock; c++) {
         curBlock.requestedParts[c / 8] |= (1 << (c % 8));
      }
   }
   requestPartialBlock = false; // this forces the AP to request the block data from the host

   while(blockAttempts--) {
      wdt10s();
#ifndef DEBUGBLOCKS
      LOG("REQ %d ", curBlock.blockId);
#else
      LOG("REQ %d[", curBlock.blockId);
      for(uint8_t c = 0; c < BLOCK_MAX_PARTS; c++) {
         if((c != 0) && (c % 8 == 0))
            LOG_RAW("][");
         if(curBlock.requestedParts[c / 8] & (1 << (c % 8))) {
            LOG_RAW("R");
         }
         else {
            LOG_RAW("_");
         }
      }
      LOG_RAW("]\n");
#endif
      struct blockRequestAck *ack = performBlockRequest();

      if(ack == NULL) {
         VLOG("Cancelled request\n");
         return false;
      }
      VLOG("pleaseWaitMs %d \n",ack->pleaseWaitMs);
      if(ack->pleaseWaitMs > 10) {
      // SLEEP - until the AP is ready with the data
         doSleep(ack->pleaseWaitMs - 10);
      }
      else {
            // immediately start with the reception of the block data
      }
      blockRxLoop(300); // BLOCK RX LOOP - receive a block, until the timeout has passed

#ifdef DEBUGBLOCKS
      LOG("RX  %d[", curBlock.blockId);
      for(uint8_t c = 0; c < BLOCK_MAX_PARTS; c++) {
         if((c != 0) && (c % 8 == 0))
            LOG_RAW("][");
         if(curBlock.requestedParts[c / 8] & (1 << (c % 8))) {
            LOG_RAW(".");
         }
         else {
            LOG_RAW("R");
         }
      }
      LOG_RAW("]\n");
#endif
   // check if we got all the parts we needed, e.g: has the block been completed?
      bool blockComplete = true;
      for(uint8_t c = 0; c < partsThisBlock; c++) {
         if(curBlock.requestedParts[c / 8] & (1 << (c % 8)))
            blockComplete = false;
      }

      if(blockComplete) {
#ifndef DEBUGBLOCKS
         LOG("- COMPLETE\n");
#endif
         if(validateBlockData()) {
            VLOG("- Validated\n");
                // block download complete, validated
            return true;
         }
         else {
            for(uint8_t c = 0; c < partsThisBlock; c++) {
               curBlock.requestedParts[c / 8] |= (1 << (c % 8));
            }
            requestPartialBlock = false;
            LOG("blk failed validation!\n");
         }
      }
      else {
#ifndef DEBUGBLOCKS
         LOG("- INCOMPLETE\n");
#endif
            // block incomplete, re-request a partial block
         requestPartialBlock = true;
      }
   }
   LOG("failed getting block\n");
   return false;
}

uint16_t dataRequestSize = 0;
static bool downloadFWUpdate(const struct AvailDataInfo *avail)
{
    // check if we already started the transfer of this information & haven't completed it
   if(!memcmp((const void *)&avail->dataVer, (const void *)&curDataInfo.dataVer, 8) && curDataInfo.dataSize) {
        // looks like we did. We'll carry on where we left off.
   }
   else {
      if(avail->dataSize > EEPROM_OTA_LEN) {
      // it would be written over the image slots behind the OTA area
         ELOG("FW size %d doesn't fit the OTA area\n",avail->dataSize);
         return false;
      }
        // start, or restart the transfer from 0. Copy data from the AvailDataInfo struct, and the struct intself. This forces a new transfer
      curBlock.blockId = 0;
      memcpy(&(curBlock.ver), &(avail->dataVer), 8);
      curBlock.type = avail->dataType;
      memcpy(&curDataInfo, (void *)avail, sizeof(struct AvailDataInfo));
      eraseUpdateBlock();
   }

   while(curDataInfo.dataSize) {
      wdt10s();
      if(curDataInfo.dataSize > BLOCK_DATA_SIZE) {
      // more than one block remaining
         dataRequestSize = BLOCK_DATA_SIZE;
      }
      else {
      // only one block remains
         dataRequestSize = curDataInfo.dataSize;
      }
      if(getDataBlock(dataRequestSize)) {
      // succesfully downloaded datablock, save to eeprom
         saveUpdateBlockData(curBlock.blockId);
         curBlock.blockId++;
         curDataInfo.dataSize -= dataRequestSize;
      }
      else {
      // failed to get the block we wanted, we'll stop for now, maybe resume later
         return false;
      }
   }
// no more data, download complete
   return true;
}

// Raw two plane images exceed 64 KB (96000 bytes on an 800x480 panel)
uint32_t imageSize = 0;

#if defined(G5_SUPPORT) || defined(ZLIB_SUPPORT)
#ifdef ZLIB_SUPPORT
#if BLOCK_DATA_SIZE < ZLIB_WINDOW_SIZE
#error "blockXferBuffer is too small for the zlib window"
#endif
#endif
// Decode the compressed (G5 or zlib) file just downloaded into slot Src into
// another slot as raw planes, leaving the displayed one alone. The
// compressed copy never gets a valid header, so its slot is simply reused
// later.
static bool decodeImage(const uint8_t Src)
{
   uint8_t Dst = findFreeSlot(Src);
   uint8_t RawType = 0;
   uint32_t RawSize;
   uint32_t SrcAdr = getAddressForSlot(Src) + sizeof(struct EepromImageHeader);
   uint32_t DstAdr;

   if(Dst == 0xFF) {
      ELOG("no slot to decode into\n");
      return false;
   }
   DstAdr = getAddressForSlot(Dst) + sizeof(struct EepromImageHeader);
   LOG("decoding type 0x%x, slot %d into slot %d\n",curDataInfo.dataType,Src,Dst);
   wdt60s();   // erasing 24 sectors and decoding take a few seconds
   eraseImageBlock(Dst);
#ifdef G5_SUPPORT
   if(curDataInfo.dataType == DATATYPE_IMG_G5) {
      RawType = g5Decode(SrcAdr,imageSize,DstAdr,&RawSize);
   }
#endif
#ifdef ZLIB_SUPPORT
   if(curDataInfo.dataType == DATATYPE_IMG_ZLIB) {
   // The block transfer buffer is idle until the next download: lend it as
   // the window
      RawType = zlibDecode(SrcAdr,imageSize,DstAdr,&RawSize,blockXferBuffer);
   }
#endif
   if(!RawType) {
      ELOG("decode failed\n");
      return false;
   }
   xferImgSlot = nextImgSlot = Dst;
   imageSize = RawSize;
   curDataInfo.dataType = RawType;
   return true;
}
#endif

static bool downloadImageDataToEEPROM(const struct AvailDataInfo *avail)
{
// check if we already started the transfer of this information & haven't completed it
   if(!memcmp((const void *)&avail->dataVer, (const void *)&curDataInfo.dataVer, 8) && curDataInfo.dataSize) {
   // looks like we did. We'll carry on where we left off.
      LOG("restarting image download to slot %d\n",xferImgSlot);
   }
   else {
      if(!avail->dataSize
         || avail->dataSize > EEPROM_IMG_EACH - sizeof(struct EepromImageHeader))
      {
         ELOG("Image size %d doesn't fit a slot\n",avail->dataSize);
         return false;
      }
      xferImgSlot = findFreeSlot(0xFF);
      if(xferImgSlot == 0xFF) {
         ELOG("no free image slot\n");
         return false;
      }
      nextImgSlot = xferImgSlot;
      drawWithLut = avail->dataTypeArgument;
      wdt10s();
      if(eepromErase(getAddressForSlot(xferImgSlot), EEPROM_IMG_EACH)) {
         ELOG("slot %d erase failed\n",xferImgSlot);
         return false;
      }
      LOG("new download, writing to slot %d\n", xferImgSlot);

   // start, or restart the transfer. Copy data from the AvailDataInfo struct, and the struct intself. This forces a new transfer
      curBlock.blockId = 0;
      memcpy(&(curBlock.ver), &(avail->dataVer), 8);
      curBlock.type = avail->dataType;
      memcpy(&curDataInfo, (void *)avail, sizeof(struct AvailDataInfo));
      imageSize = curDataInfo.dataSize;
   }

   while(curDataInfo.dataSize) {
      wdt10s();
      if(curDataInfo.dataSize > BLOCK_DATA_SIZE) {
      // more than one block remaining
         dataRequestSize = BLOCK_DATA_SIZE;
      }
      else {
      // only one block remains
         dataRequestSize = curDataInfo.dataSize;
      }
      if(getDataBlock(dataRequestSize)) {
      // succesfully downloaded datablock, save to eeprom
         LOG("Saving block %d to slot %d\n", curBlock.blockId, xferImgSlot);
         saveImgBlockData(xferImgSlot, curBlock.blockId);
         curBlock.blockId++;
         curDataInfo.dataSize -= dataRequestSize;
      }
      else {
      // failed to get the block we wanted, we'll stop for now, probably resume later
         return false;
      }
   }
// no more data, download complete
#if defined(G5_SUPPORT) || defined(ZLIB_SUPPORT)
   if((false
#ifdef G5_SUPPORT
       || curDataInfo.dataType == DATATYPE_IMG_G5
#endif
#ifdef ZLIB_SUPPORT
       || curDataInfo.dataType == DATATYPE_IMG_ZLIB
#endif
      ) && !decodeImage(xferImgSlot))
   {
   // Forget the transfer: with dataSize 0 and a matching version the next
   // AvailDataInfo would otherwise be taken for the image on display
      memset(&curDataInfo,0,sizeof(curDataInfo));
      return false;
   }
#endif

// borrow the blockXferBuffer temporarily
   struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
   memset(eih,0,sizeof(struct EepromImageHeader));   // no stale padding bytes
   memcpy(&eih->version, &curDataInfo.dataVer, 8);
   eih->validMarker = EEPROM_IMG_VALID;
   eih->id = ++curHighSlotId;
   eih->size = imageSize;
   eih->dataType = curDataInfo.dataType;
   eih->argument = curDataInfo.dataTypeArgument;

#ifdef DEBUGBLOCKS
   LOG("Now writing datatype 0x%02X to slot %d\n", curDataInfo.dataType, xferImgSlot);
#endif
   eepromWrite(getAddressForSlot(xferImgSlot), eih, sizeof(struct EepromImageHeader));
#ifdef OEPL_PRELOADS
   removeOldSpecial(curDataInfo.dataTypeArgument,xferImgSlot);
#endif

   return true;
}

bool processAvailDataInfo(struct AvailDataInfo *avail)
{
   uint8_t Slot;

   LOG("dataType: 0x%x\n", avail->dataType);
   switch(avail->dataType) {
      case DATATYPE_IMG_BMP:
      case DATATYPE_IMG_DIFF:
         break;
      case DATATYPE_IMG_RAW_1BPP:
      case DATATYPE_IMG_RAW_2BPP:
      case DATATYPE_IMG_ZLIB:
#ifdef G5_SUPPORT
      case DATATYPE_IMG_G5:
#endif
         LOG("RAW_BPP\n");
            // check if this download is currently displayed or active
         if(curDataInfo.dataSize == 0 && !memcmp((const void *)&avail->dataVer, (const void *)&curDataInfo.dataVer, 8)) {
                // we've downloaded this already, we're guessing it's already displayed
            LOG("currently shown image, send xfc\n");
            sendXferComplete();
            return true;
         }
#ifdef OEPL_PRELOADS
         if(avail->dataTypeArgument & ARG_PRELOAD) {
         // Store it for later (special type: AP lost screen etc.), don't draw.
         // The same pixels can come with another role, so a cached copy has
         // to match the argument too.
            struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;

            Slot = findSlot((uint8_t *)&(avail->dataVer));
            if(Slot != 0xFF) {
               eepromRead(getAddressForSlot(Slot),eih,sizeof(struct EepromImageHeader));
               if(eih->argument == avail->dataTypeArgument) {
                  LOG("preload already stored in slot %d\n",Slot);
                  sendXferComplete();
                  return true;
               }
            }
            LOG("preloading type %d\n",ARG_SPECIAL(avail->dataTypeArgument));
            if(!downloadImageDataToEEPROM(avail)) {
               return false;
            }
         // Not on display: if the same pixels come later as the regular
         // image, the check above must not take them for the one shown
            memset(&curDataInfo,0,sizeof(curDataInfo));
            sendXferComplete();
            return true;
         }
#endif

      // check if we've seen this version before
         Slot = findSlot((uint8_t *)&(avail->dataVer));
         if(Slot != 0xFF) {
            curImgSlot = Slot;
         // found a (complete)valid image slot for this version
            LOG("already seen, drawing from eeprom slot %d\n", curImgSlot);

                // mark as completed and draw from EEPROM
            memcpy(&curDataInfo, (void *)avail, sizeof(struct AvailDataInfo));
            curDataInfo.dataSize = 0; // mark as transfer not pending

            drawWithLut = avail->dataTypeArgument;
            wdt60s();
            drawOnOffline(1);
            drawImageFromEeprom(curImgSlot);
         // XferComplete after the refresh, with the panel off: sent during the
         // refresh current spike it got lost about half the time (CC1110
         // Chroma tags)
            sendXferComplete();
            return true;
         }
         else {
         // not found in cache, prepare to download
            LOG("downloading to imgslot\n");
            drawWithLut = avail->dataTypeArgument;
            if(downloadImageDataToEEPROM(avail)) {
               LOG("download complete!\n");
               curImgSlot = xferImgSlot;
               wdt60s();
               drawOnOffline(1);
               drawImageFromEeprom(curImgSlot);
               sendXferComplete();   // after the refresh, see above
               return true;
            }
            else {
               return false;
            }
         }
         break;
      case DATATYPE_FW_UPDATE:
         if(downloadFWUpdate(avail)) {
            sendXferComplete();
            LOG("firmware download complete, doing update.\n");
            write_ota_firmware_to_flash();
         }
         else {
            return false;
         }
         break;
#ifdef OEPL_COMMANDS
      case DATATYPE_COMMAND_DATA:
      // avail points into the radio buffer, which the XferComplete ACK
      // overwrites: take the command out first
         Slot = avail->dataTypeArgument;
         LOG("CMD 0x%x\n",Slot);
         sendXferComplete();   // before: reboot and deep sleep don't return
         executeCommand(Slot);
         break;
#endif
      case DATATYPE_NFC_URL_DIRECT:
      case DATATYPE_NFC_RAW_CONTENT:
         break;
      case DATATYPE_CUSTOM_LUT_OTA:
         break;
         return true;
   }
   return true;
}

#ifndef OEPL_OTA
void write_ota_firmware_to_flash(void)
{
   LOG("Called\n");
}
#endif

void initializeProto()
{
   curHighSlotId = getHighSlotId();
}

