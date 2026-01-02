#ifndef KEY_H
#define KEY_H
#include "stm32f1xx_hal.h"

uint8_t getKeyPressFlag(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
