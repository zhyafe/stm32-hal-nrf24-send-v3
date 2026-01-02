#include "key.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>

// 定义引脚（同前）
#define KEY_PIN GPIO_PIN_13
#define KEY_GPIO_PORT GPIOB

// 按键状态变量
typedef enum {
  KEY_IDLE,    // 空闲（未按下）
  KEY_PRESSED, // 已按下（消抖后确认）
  KEY_RELEASED // 已释放（消抖后确认）
} KeyState;

KeyState key_state = KEY_IDLE;
uint32_t key_press_time = 0;  // 记录按键状态变化的时间戳
uint32_t debounce_delay = 20; // 消抖延时（ms）

uint8_t getKeyPressFlag(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  uint8_t pressFlag = 0;

  // 1. 非阻塞式按键检测
  uint8_t current_level = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  uint32_t current_time = HAL_GetTick(); // 获取系统运行时间（ms）

  switch (key_state) {
  case KEY_IDLE:
    // 检测到按键按下（低电平），进入消抖
    if (current_level == GPIO_PIN_RESET) {
      key_press_time = current_time;
      key_state = KEY_PRESSED;
    }
    break;

  case KEY_PRESSED:
    // 消抖：持续低电平超过20ms，确认按下
    if (current_level == GPIO_PIN_RESET &&
        (current_time - key_press_time) >= debounce_delay) {
      key_state = KEY_RELEASED; // 等待释放
    } else if (current_level == GPIO_PIN_SET) {
      // 未满足消抖时间就松开，视为误触，回到空闲
      key_state = KEY_IDLE;
    }
    break;

  case KEY_RELEASED:
    // 检测到按键释放（高电平），回到空闲
    if (current_level == GPIO_PIN_SET) {
      key_state = KEY_IDLE;
      pressFlag = 1;
    }
    break;
  }

  return pressFlag;
}
