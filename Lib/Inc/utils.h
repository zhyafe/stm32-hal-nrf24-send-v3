#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

uint16_t mapVal(uint16_t sourceMin, uint16_t sourceMax, uint16_t targetMin,
                uint16_t targetMax, uint16_t value);

#endif