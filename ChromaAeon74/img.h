#ifndef IMG_H
#define IMG_H

#include "img.h"
#include <stdint.h>

#define IMG_HEIGHT 320
#define IMG_WIDTH 320
#define IMG_DATA_SIZE (IMG_HEIGHT * IMG_WIDTH / 8)

extern const uint8_t img_data[IMG_DATA_SIZE];

#endif
