#ifndef _EPD_74H_H_
#define _EPD_74H_H_

#include <stdbool.h>
#include <stdint.h>

#define EPD_WIDTH       800
#define EPD_HEIGHT      480
#define EPD_PLANE_SIZE  (EPD_WIDTH * EPD_HEIGHT / 8)

// EpdDrawFromFlash() flags
#define EPD_INVERT_BW   0x01  // plane 1 is 1 = black (OEPL), panel wants 1 = white
#define EPD_INVERT_Y    0x02  // plane 2 is 1 = yellow (OEPL), panel wants 0 = yellow
#define EPD_NO_YELLOW   0x04  // B/W only image: send an empty yellow plane

#define EPD_ROW_BYTES   (EPD_WIDTH / 8)
// EpdDrawFromFlash() PlaneAdr for a blank (white) screen, e.g. to draw a
// layer on
#define EPD_NO_IMAGE    0xffffffff

// Draw an image stored in SPI flash at PlaneAdr: the B/W plane followed by
// the yellow plane, EPD_PLANE_SIZE bytes each. With Flags = 0 the bytes go
// to the panel as stored (the stock screen slot layout). Runs the panel's
// factory command script. Returns true on error.
bool EpdDrawFromFlash(uint32_t PlaneAdr,uint8_t Flags);

// A layer of filled rectangles and 8x8 text (scaled by Scale) drawn over the
// image row by row while it is sent: no frame buffer needed. Rectangles are
// drawn first, then texts.
#define EPD_INK_BLACK   0
#define EPD_INK_YELLOW  1
#define EPD_INK_WHITE   2

typedef struct {
   uint16_t X;
   uint16_t Y;
   uint16_t W;
   uint16_t H;
   uint8_t Ink;
} EpdRect;

typedef struct {
   uint16_t X;
   uint16_t Y;
   uint8_t Scale;
   uint8_t Ink;
   const char *pText;
} EpdText;

typedef struct {
   const EpdRect *pRects;
   uint8_t Rects;
   const EpdText *pTexts;
   uint8_t Texts;
} EpdLayer;

// Layer for the following draws, NULL for none
void EpdSetLayer(const EpdLayer *pLayer);

// Lowest supply voltage (mV) during the last draw's busy waits, 0 = none
extern uint16_t gEpdRefreshMinMv;

#ifdef EPD_TEST
void EpdTest(void);
#endif

#endif   // _EPD_74H_H_
