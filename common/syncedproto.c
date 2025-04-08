#include <stdint.h>
#include <string.h>

#include "main.h"
#include "syncedproto.h"
#include "oepl-proto.h"
#include "oepl-definitions.h"
#include "powermgt.h"
#include "eeprom.h"
#include "drawing.h"
#include "board.h"
#include "radio.h"
#include "logging.h"

// download-stuff
uint8_t blockXferBuffer[BLOCK_DATA_SIZE];
struct blockRequest curBlock = {0};     // used by the block-requester, contains the next request that we'll send
struct AvailDataInfo curDataInfo = {0}; // last 'AvailDataInfo' we received from the AP
bool requestPartialBlock = false;       // if we should ask the AP to get this block from the host or not
#define BLOCK_TRANSFER_ATTEMPTS 5

uint8_t prevImgSlot = 0xFF;
uint8_t curImgSlot = 0xFF;
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
   if((fcs->frameType == 1) && (fcs->destAddrType == 2) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 0)) {
        // broadcast frame
      uint8_t type = ((uint8_t *)buffer)[sizeof(struct MacFrameBcast)];
      return type;
   }
   else if((fcs->frameType == 1) && (fcs->destAddrType == 3) && (fcs->srcAddrType == 3) && (fcs->panIdCompressed == 1)) {
        // normal frame
      uint8_t type = ((uint8_t *)buffer)[sizeof(struct MacFrameNormal)];
      return type;
   }
   return 0;
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
    // validate if it's okay to copy data
   if(bp->blockId != curBlock.blockId) {
        // LOG("got a packet for block %02X\n", bp->blockId);
      return false;
   }
   if(start >= (sizeof(blockXferBuffer) - 1))
      return false;
   if(bp->blockPart > BLOCK_MAX_PARTS)
      return false;
   if((start + size) > sizeof(blockXferBuffer)) {
      size = sizeof(blockXferBuffer) - start;
   }
   if(checkCRC(bp, sizeof(struct blockPart) + BLOCK_PART_DATA_SIZE)) {
        //  copy block data to buffer
      memcpy((void *)(blockXferBuffer + start), (const void *)bp->data, size);
        // we don't need this block anymore, set bit to 0 so we don't request it again
      curBlock.requestedParts[bp->blockPart / 8] &= ~(1 << (bp->blockPart % 8));
      return true;
   }
   else {
      ELOG("CRC Failed\n");
      return false;
   }
}

static bool blockRxLoop(const uint32_t timeout)
{
   bool success = false;
    // radioRxEnable(true);
   uint8_t *inBuffer = commsRxUnencrypted(timeout);
   if(inBuffer != NULL) {
      if(getPacketType(inBuffer) == PKT_BLOCK_PART) {
         struct blockPart *bp = (struct blockPart *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
         success = processBlockPart(bp);
      }
   }
    // radioRxEnable(false);
    // radioRxFlush();
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
         switch(getPacketType(inBuffer)) {
            case PKT_BLOCK_REQUEST_ACK:
               if(checkCRC((inBuffer + sizeof(struct MacFrameNormal) + 1), sizeof(struct blockRequestAck))) {
                  pRet = (struct blockRequestAck *)(inBuffer + sizeof(struct MacFrameNormal) + 1);
               }
               break;

            case PKT_BLOCK_PART:
            // block already started while we were waiting for a get block reply
            // LOG("!");
            // processBlockPart((struct blockPart *)(inBuffer + sizeof(struct MacFrameNormal) + 1));
               pRet = continueToRX(inBuffer);
               break;

            case PKT_CANCEL_XFER:
               return NULL;

            default:
               LOG("pkt w/type %02X\n", getPacketType(inBuffer));
               break;
         }
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
   commsTxNoCpy(outBuffer);
}

static void sendXferComplete()
{
   radioRxEnable(true);

   for(uint8_t c = 0; c < 16; c++) {
      sendXferCompletePacket();
      uint8_t *inBuffer = commsRxUnencrypted(6);
      if(inBuffer != NULL) {
         if(getPacketType(inBuffer) == PKT_XFER_COMPLETE_ACK) {
            LOG("XFC ACK\n");
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
   LOG("expected len = %04X, checksum=%04X\n", bd->size, bd->checksum);
   if(bd->size > BLOCK_XFER_BUFFER_SIZE - sizeof(struct blockData)) {
      LOG("Impossible data size, we abort here\n");
      return false;
   }
   uint16_t t = 0;
   for(uint16_t c = 0; c < bd->size; c++) {
      t += bd->data[c];
   }
   LOG("Checked len = %04X, checksum=%04X\n", bd->size, t);
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

static void saveUpdateBlockData(uint8_t blockId)
{
   if(!eepromWrite(EEPROM_OTA_START+ (blockId * BLOCK_DATA_SIZE), blockXferBuffer + sizeof(struct blockData), BLOCK_DATA_SIZE))
      LOG("EEPROM write failed\n");
}

static void saveImgBlockData(const uint8_t imgSlot, const uint8_t blockId)
{
   uint32_t length = EEPROM_IMG_EACH - (sizeof(struct EepromImageHeader) + (blockId * BLOCK_DATA_SIZE));
   if(length > 4096)
      length = 4096;

   if(!eepromWrite(getAddressForSlot(imgSlot) + sizeof(struct EepromImageHeader) + (blockId * BLOCK_DATA_SIZE), blockXferBuffer + sizeof(struct blockData), length))
      LOG("EEPROM write failed\n");
}

void drawImageFromEeprom(const uint8_t imgSlot)
{
   drawImageAtAddress(getAddressForSlot(imgSlot), drawWithLut);
   drawWithLut = 0; // default back to the regular ol' stock/OTP LUT
}

static uint32_t getHighSlotId()
{
   uint32_t temp = 0;
   uint32_t markerValid = EEPROM_IMG_VALID;
   for(uint8_t c = 0; c < IMAGE_SLOTS; c++) {
      struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
      eepromRead(getAddressForSlot(c),eih,sizeof(struct EepromImageHeader));
      if(!memcmp(&eih->validMarker, &markerValid, 4)) {
         if(temp < eih->id) {
            temp = eih->id;
            nextImgSlot = c;
         }
      }
   }
   LOG("found high id=%d in slot %d\n", temp, nextImgSlot);
   return temp;
}

static uint8_t partsThisBlock = 0;
static uint8_t blockAttempts = 0; // these CAN be local to the function, but for some reason, they won't survive sleep?
                                  // they get overwritten with  7F 32 44 20 00 00 00 00 11, I don't know why.

static bool getDataBlock(const uint16_t blockSize)
{
   blockAttempts = BLOCK_TRANSFER_ATTEMPTS;
   if(blockSize == BLOCK_DATA_SIZE) {
      partsThisBlock = BLOCK_MAX_PARTS;
      memset(curBlock.requestedParts, 0xFF, BLOCK_REQ_PARTS_BYTES);
   }
   else {
      partsThisBlock = (sizeof(struct blockData) + blockSize) / BLOCK_PART_DATA_SIZE;
      if((sizeof(struct blockData) + blockSize) % BLOCK_PART_DATA_SIZE)
         partsThisBlock++;
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
            LOG("][");
         if(curBlock.requestedParts[c / 8] & (1 << (c % 8))) {
            LOG("R");
         }
         else {
            LOG("_");
         }
      }
      LOG("]\n");
#endif
      struct blockRequestAck *ack = performBlockRequest();

      if(ack == NULL) {
         LOG("Cancelled request\n");
         return false;
      }
      if(ack->pleaseWaitMs) { // SLEEP - until the AP is ready with the data
         WaitMs(ack->pleaseWaitMs - 10);
      }
      else {
            // immediately start with the reception of the block data
      }
      blockRxLoop(300); // BLOCK RX LOOP - receive a block, until the timeout has passed

#ifdef DEBUGBLOCKS
      LOG("RX  %d[", curBlock.blockId);
      for(uint8_t c = 0; c < BLOCK_MAX_PARTS; c++) {
         if((c != 0) && (c % 8 == 0))
            LOG("][");
         if(curBlock.requestedParts[c / 8] & (1 << (c % 8))) {
            LOG(".");
         }
         else {
            LOG("R");
         }
      }
      LOG("]\n");
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
            LOG("- Validated\n");
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

uint16_t imageSize = 0;
static bool downloadImageDataToEEPROM(const struct AvailDataInfo *avail)
{
// check if we already started the transfer of this information & haven't completed it
   if(!memcmp((const void *)&avail->dataVer, (const void *)&curDataInfo.dataVer, 8) && curDataInfo.dataSize) {
   // looks like we did. We'll carry on where we left off.
      LOG("restarting image download");
      curImgSlot = nextImgSlot;
   }
   else {
   // go to the next image slot
      nextImgSlot++;
      if(nextImgSlot >= IMAGE_SLOTS)
         nextImgSlot = 0;
      curImgSlot = nextImgSlot;
      LOG("Saving to image slot %d\n", curImgSlot);
      drawWithLut = avail->dataTypeArgument;
      uint8_t attempt = 5;
      while(attempt--) {
         if(eepromErase(getAddressForSlot(curImgSlot), EEPROM_IMG_EACH))
            goto eraseSuccess;
      }
   // eepromFail:
      doSleep(-1);
      eraseSuccess:
      LOG("new download, writing to slot %d\n", curImgSlot);

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
         LOG("Saving block %d to slot %d\n", curBlock.blockId, curImgSlot);
         saveImgBlockData(curImgSlot, curBlock.blockId);
         curBlock.blockId++;
         curDataInfo.dataSize -= dataRequestSize;
      }
      else {
      // failed to get the block we wanted, we'll stop for now, probably resume later
         return false;
      }
   }
// no more data, download complete

// borrow the blockXferBuffer temporarily
   struct EepromImageHeader *eih = (struct EepromImageHeader *)blockXferBuffer;
   memcpy(&eih->version, &curDataInfo.dataVer, 8);
   eih->validMarker = EEPROM_IMG_VALID;
   eih->id = ++curHighSlotId;
   eih->size = imageSize;
   eih->dataType = curDataInfo.dataType;

#ifdef DEBUGBLOCKS
   LOG("Now writing datatype 0x%02X to slot %d\n", curDataInfo.dataType, curImgSlot);
#endif
   eepromWrite(getAddressForSlot(curImgSlot), eih, sizeof(struct EepromImageHeader));

   return true;
}

bool processAvailDataInfo(struct AvailDataInfo *avail)
{
   LOG("dataType: %d\n", avail->dataType);
   switch(avail->dataType) {
      case DATATYPE_IMG_BMP:
      case DATATYPE_IMG_DIFF:
         break;
      case DATATYPE_IMG_RAW_1BPP:
      case DATATYPE_IMG_RAW_2BPP:
         LOG("RAW_BPP\n");
            // check if this download is currently displayed or active
         if(curDataInfo.dataSize == 0 && !memcmp((const void *)&avail->dataVer, (const void *)&curDataInfo.dataVer, 8)) {
                // we've downloaded this already, we're guessing it's already displayed
            LOG("currently shown image, send xfc\n");
            sendXferComplete();
            return true;
         }

            // check if we've seen this version before
         curImgSlot = findSlot((uint8_t *)&(avail->dataVer));
         if(curImgSlot != 0xFF) {
                // found a (complete)valid image slot for this version
            sendXferComplete();

            LOG("already seen, drawing from eeprom slot %d\n", curImgSlot);

                // mark as completed and draw from EEPROM
            memcpy(&curDataInfo, (void *)avail, sizeof(struct AvailDataInfo));
            curDataInfo.dataSize = 0; // mark as transfer not pending

            drawWithLut = avail->dataTypeArgument;
            wdt60s();
            drawOnOffline(1);
            drawImageFromEeprom(curImgSlot);
            return true;
         }
         else {
                // not found in cache, prepare to download
            LOG("downloading to imgslot\n");
            drawWithLut = avail->dataTypeArgument;
            if(downloadImageDataToEEPROM(avail)) {
               LOG("download complete!\n");
               sendXferComplete();

               wdt60s();
               drawOnOffline(1);
               drawImageFromEeprom(curImgSlot);
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
      case DATATYPE_NFC_URL_DIRECT:
      case DATATYPE_NFC_RAW_CONTENT:
         break;
      case DATATYPE_CUSTOM_LUT_OTA:
         break;
         return true;
   }
   return true;
}

uint32_t address = 0;

void write_ota_firmware_to_flash(void)
{
   LOG("Called\n");
}

void initializeProto()
{
   curHighSlotId = getHighSlotId();
}

