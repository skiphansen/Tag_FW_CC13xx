#ifndef _ZLIB_DECODE_H_
#define _ZLIB_DECODE_H_
#include <stdint.h>

// OEPL's DATATYPE_IMG_ZLIB images hold a u32 (the uncompressed size), then
// a zlib stream of the six-byte image header (6, width, height, planes) and
// the planes. The AP compresses with a 4 KB dictionary and says so in the
// zlib header.
#define ZLIB_WINDOW_SIZE   4096

// Inflate the file in SPI flash at Source (Size bytes) into the erased area
// at Destination, as raw planes like the ones the AP sends uncompressed.
// pWindow is ZLIB_WINDOW_SIZE bytes of scratch RAM, only used during the
// call. Returns DATATYPE_IMG_RAW_1BPP or _2BPP with the raw size in
// *pRawSize, or 0 on failure.
uint8_t zlibDecode(uint32_t Source,uint32_t Size,uint32_t Destination,
                   uint32_t *pRawSize,uint8_t *pWindow);
#endif
