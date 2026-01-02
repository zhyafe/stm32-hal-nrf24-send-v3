#include "oled.h"
#include "font.h" // 字库数据
#include "i2c.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>

// OLED显存
static uint8_t OLED_GRAM[128][8];

uint8_t OLED_UPDATE_ROWS[8];

// 向OLED发送命令
void OLED_Write_Cmd(uint8_t cmd) {
  // HAL_I2C_Mem_Write(&hi2c1, OLED_ADDRESS, 0x00, 1, &cmd, 1, 100);
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, (uint8_t[]){0x00, cmd}, 2, 100);
}

// 向OLED发送数据
void OLED_Write_Data(uint8_t data) {
  // HAL_I2C_Mem_Write(&hi2c1, OLED_ADDRESS, 0x40, 1, &data, 1, 100);
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, (uint8_t[]){0x40, data}, 2,
                          100);
}

// OLED初始化
void OLED_Init(void) {
  HAL_Delay(100);

  // 初始化序列
  OLED_Write_Cmd(0xAE); // 关闭显示
  OLED_Write_Cmd(0x20); // 设置内存地址模式
  OLED_Write_Cmd(0x10); // 页地址模式
  OLED_Write_Cmd(0xB0); // 设置页起始地址
  OLED_Write_Cmd(0xC8); // 设置扫描方向
  OLED_Write_Cmd(0x00); // 设置列低地址
  OLED_Write_Cmd(0x10); // 设置列高地址
  OLED_Write_Cmd(0x40); // 设置显示起始行
  OLED_Write_Cmd(0x81); // 设置对比度
  OLED_Write_Cmd(0xFF); // 对比度值
  OLED_Write_Cmd(0xA1); // 设置段重映射
  OLED_Write_Cmd(0xA6); // 正常显示
  OLED_Write_Cmd(0xA8); // 设置多路复用率
  OLED_Write_Cmd(0x3F); // 1/64 duty
  OLED_Write_Cmd(0xA4); // 全部显示开启
  OLED_Write_Cmd(0xD3); // 设置显示偏移
  OLED_Write_Cmd(0x00); // 无偏移
  OLED_Write_Cmd(0xD5); // 设置振荡器频率
  OLED_Write_Cmd(0xF0); // 设置分频比
  OLED_Write_Cmd(0xD9); // 设置预充电周期
  OLED_Write_Cmd(0x22); //
  OLED_Write_Cmd(0xDA); // 设置COM硬件引脚配置
  OLED_Write_Cmd(0x12); //
  OLED_Write_Cmd(0xDB); // 设置VCOMH
  OLED_Write_Cmd(0x20); //
  OLED_Write_Cmd(0x8D); // 设置充电泵
  OLED_Write_Cmd(0x14); // 启用充电泵
  OLED_Write_Cmd(0xAF); // 开启显示

  OLED_Clear();
  memset(OLED_UPDATE_ROWS, 1, 8);
  OLED_Refresh();
}

// 清屏
void OLED_Clear(void) {
  memset(OLED_UPDATE_ROWS, 0, 8);
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 128; j++) {
      OLED_GRAM[j][i] = 0x00;
    }
  }
}

// 刷新显存到OLED
void OLED_Refresh(void) {
  for (int i = 0; i < 8; i++) {
    if (OLED_UPDATE_ROWS[i] == 0)
      continue;               // 该行未更新，跳过
    OLED_Write_Cmd(0xB0 + i); // 设置页地址
    OLED_Write_Cmd(0x00);     // 设置列低地址
    OLED_Write_Cmd(0x10);     // 设置列高地址

    // for (int j = 0; j < 128; j++) {
    //   OLED_Write_Data(OLED_GRAM[j][i]);
    // }

    // 一次性发送一页数据（129字节，首字节为0x40数据标志）
    uint8_t page_buf[129];
    page_buf[0] = 0x40; // 数据标志
    for (int j = 0; j < 128; j++) {
      page_buf[j + 1] = OLED_GRAM[j][i];
    }
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, page_buf, 129, 1000);
  }
}

// 显示一个字符
void OLED_ShowChar(uint8_t x, uint8_t y, char chr) {
  if (chr < 32 || chr > 127)
    return; // 只支持ASCII 32-127

  OLED_UPDATE_ROWS[y] = 1;
  for (int i = 0; i < 6; i++) {
    uint8_t temp = ASCII_Font[chr - 32][i];
    OLED_GRAM[x + i][y] = temp;
  }
}

// 显示字符串
void OLED_ShowString(uint8_t x, uint8_t y, char *str) {
  OLED_UPDATE_ROWS[y] = 1;
  while (*str) {
    OLED_ShowChar(x, y, *str++);
    x += 6;
    if (x > 122) {
      x = 0;
      y++;
    }
  }
}

// 显示数字
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num) {
  OLED_UPDATE_ROWS[y] = 1;
  char str[10];
  // snprintf(str, sizeof(str), "%x", num); // 十六进制
  snprintf(str, sizeof(str), "%d", num); // 十进制
  OLED_ShowString(x, y, str);
}

// 显示数字
void OLED_ShowFloat(uint8_t x, uint8_t y, float num) {
  OLED_UPDATE_ROWS[y] = 1;
  int int_part = (int)num;
  int frac_part = (int)((num - int_part) * 100); // 保留两位小数
  if (frac_part < 0)
    frac_part = -frac_part;

  char str[20];
  snprintf(str, sizeof(str), "%d.%02d", int_part, frac_part);
  OLED_ShowString(x, y, str);
}
