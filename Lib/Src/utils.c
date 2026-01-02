#include <stdint.h>

uint16_t mapVal(uint16_t sourceMin, uint16_t sourceMax, uint16_t targetMin,
                uint16_t targetMax, uint16_t value) {
  uint16_t mappedValue = targetMin + (value - sourceMin) *
                                         (targetMax - targetMin) /
                                         (sourceMax - sourceMin);
  return mappedValue;
}
