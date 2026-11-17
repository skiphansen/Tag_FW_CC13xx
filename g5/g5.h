#ifndef G5_H
#define G5_H
/* Packed-row Group5 decoder from Tag_FW_Chroma (g5/ at 0e17be9), by Larry
 * Bank (BitBank Software); see ./LICENSE and ./APL.txt. CC13xx changes: rows
 * up to 800 pixels, a private workspace instead of Chroma's imageScratch
 * union, transfer parameters passed to g5Decode(). */
#include <stdint.h>
#ifndef __SDCC
#define __xdata
#endif

/* A single non-reentrant decoder owns the workspace. */
#define G5_MAX_WIDTH 800
#define G5_ROW_BYTES 100
#define G5_INPUT_BYTES 128

typedef struct {
    uint16_t width, height, line;
    uint8_t pitch, longbits;
    uint8_t __xdata *cur, *ref;
    uint8_t byte, mask;
    int8_t error;
} G5State;
typedef struct {
    uint16_t e1, e2, e3, b2, start, second;
    uint8_t white, refwhite;
    int8_t mode;
    uint16_t br_start, br_end;
    uint8_t br_left, br_right, br_tail;
} G5Work;
typedef struct {
    uint32_t source, remaining, destination;
    uint16_t width, height;
    uint8_t pos, count;
    uint32_t rawSize;
} G5Transfer;

typedef struct {
    uint8_t rows[2][G5_ROW_BYTES];
    G5State state;
    G5Work work;
    G5Transfer transfer;
    uint8_t input[G5_INPUT_BYTES];
} G5Workspace;

/* Source is the compressed file (including the six-byte header) in SPI
 * flash, Size its length, Destination an erased area for the raw planes.
 * Returns a raw image data type and the raw size in *pRawSize, or zero on
 * failure. Writes neither image-valid markers nor protocol acknowledgements. */
uint8_t g5Decode(uint32_t Source,uint32_t Size,uint32_t Destination,uint32_t *pRawSize);
#ifdef G5_HOST_TEST
int8_t g5p_init(uint16_t width, uint16_t height,
                uint8_t __xdata *row0, uint8_t __xdata *row1);
int8_t g5p_decode_line(void);
uint8_t __xdata *g5p_row(void);
#endif
#endif
