#ifndef _RGB2YUV_CUH_
#define _RGB2YUV_CUH_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

extern "C" void convert_rgb_to_yu12(uint8_t *input, uint8_t *output);

#endif