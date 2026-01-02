#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"

#define OLED_ADDRESS 0x78 // OLED I2C地址

// OLED控制函数
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num);
void OLED_Refresh(void);

#endif
