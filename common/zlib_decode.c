/*
 * DATATYPE_IMG_ZLIB images decoded from SPI flash into a raw image slot, so
 * the panel driver only ever draws raw planes. The CC1310's 16 KB RAM has
 * no room for a dedicated window: the caller lends one (the block transfer
 * buffer, idle once a download is complete).
 */
#include "oepl_board.h"

#ifdef ZLIB_SUPPORT
#include <stdbool.h>
#include <string.h>
#include "uzlib.h"
#include "eeprom.h"
#include "oepl-proto.h"
#include "oepl-definitions.h"
#include "zlib_decode.h"
#include "logging.h"

#define ZLIB_IN_SIZE       128
#define ZLIB_OUT_SIZE      128

static struct {
   struct uzlib_uncomp lib;
   uint32_t Source;           // next SPI flash address to read
   uint32_t Remaining;        // compressed bytes not read yet
   uint8_t In[ZLIB_IN_SIZE];
   uint8_t Out[ZLIB_OUT_SIZE];
} gZ;

// uzlib asks for more input: refill In from SPI flash, return its first byte
static int ZlibReadCB(struct uzlib_uncomp *pLib)
{
   uint32_t Len = gZ.Remaining;

   if(Len == 0) {
      return -1;
   }
   if(Len > ZLIB_IN_SIZE) {
      Len = ZLIB_IN_SIZE;
   }
   if(eepromRead(gZ.Source,gZ.In,Len)) {
      return -1;
   }
   gZ.Source += Len;
   gZ.Remaining -= Len;
   pLib->source = &gZ.In[1];
   pLib->source_limit = &gZ.In[Len];
   return gZ.In[0];
}

// Inflate exactly Len bytes into gZ.Out. Returns the uzlib result
// (TINF_OK, or TINF_DONE when the stream ended, checksum verified).
static int ZlibRead(uint32_t Len)
{
   gZ.lib.dest_start = gZ.lib.dest = gZ.Out;
   gZ.lib.dest_limit = gZ.Out + Len;
   return uzlib_uncompress_chksum(&gZ.lib);
}

uint8_t zlibDecode(uint32_t Source,uint32_t Size,uint32_t Destination,
                   uint32_t *pRawSize,uint8_t *pWindow)
{
   uint32_t RawSize;
   uint32_t Done;
   uint32_t Len;
   uint16_t Width;
   uint16_t Height;
   uint8_t Planes;
   int Res;

   if(Size < 4 + 2 + 6 + 4) {
      ELOG("zlib: file too short\n");
      return 0;
   }
// Skip the u32 uncompressed size: the image header below says the same
   memset(&gZ.lib,0,sizeof(gZ.lib));
   gZ.Source = Source + 4;
   gZ.Remaining = Size - 4;
   uzlib_init();
   uzlib_uncompress_init(&gZ.lib,pWindow,ZLIB_WINDOW_SIZE);
   gZ.lib.source = NULL;
   gZ.lib.source_limit = NULL;
   gZ.lib.source_read_cb = ZlibReadCB;

// The window the stream was made with must fit ours (uzlib also refuses any
// match reaching further back than ZLIB_WINDOW_SIZE)
   Res = uzlib_zlib_parse_header(&gZ.lib);
   if(Res < 0 || (0x100 << Res) > ZLIB_WINDOW_SIZE) {
      ELOG("zlib: bad header or window (%d)\n",Res);
      return 0;
   }

   Res = ZlibRead(6);
   if(Res < 0 || gZ.lib.dest != gZ.Out + 6) {
      ELOG("zlib: no image header (%d)\n",Res);
      return 0;
   }
   Width = gZ.Out[1] | (gZ.Out[2] << 8);
   Height = gZ.Out[3] | (gZ.Out[4] << 8);
   Planes = gZ.Out[5];
   if(gZ.Out[0] != 6 || Width != SCREEN_WIDTH || Height != SCREEN_HEIGHT
      || (Planes != 1 && Planes != 2))
   {
      ELOG("zlib: image %u x %u, %u planes not supported\n",Width,Height,Planes);
      return 0;
   }
   RawSize = (uint32_t) (SCREEN_WIDTH / 8) * SCREEN_HEIGHT * Planes;
   if(RawSize > EEPROM_IMG_EACH - sizeof(struct EepromImageHeader)) {
      return 0;
   }

   for(Done = 0; Done < RawSize; Done += Len) {
      Len = RawSize - Done;
      if(Len > ZLIB_OUT_SIZE) {
         Len = ZLIB_OUT_SIZE;
      }
      Res = ZlibRead(Len);
      if(Res < 0 || gZ.lib.dest != gZ.Out + Len) {
         ELOG("zlib: inflate failed at %u (%d)\n",Done,Res);
         return 0;
      }
      if(eepromWrite(Destination + Done,gZ.Out,Len)) {
         ELOG("zlib: SPI flash write failed\n");
         return 0;
      }
   }
// The stream must end here, with a good Adler-32
   if(Res != TINF_DONE) {
      Res = ZlibRead(0);
   }
   if(Res != TINF_DONE) {
      ELOG("zlib: stream doesn't end with the image (%d)\n",Res);
      return 0;
   }
   *pRawSize = RawSize;
   return Planes == 1 ? DATATYPE_IMG_RAW_1BPP : DATATYPE_IMG_RAW_2BPP;
}
#endif   // ZLIB_SUPPORT
