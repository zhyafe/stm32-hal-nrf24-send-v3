#ifndef __NRF24_H
#define __NRF24_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

// NRF24L01引脚定义
#define NRF24_CE_PORT    GPIOA
#define NRF24_CE_PIN     GPIO_PIN_3
#define NRF24_CS_PORT    GPIOA
#define NRF24_CS_PIN     GPIO_PIN_4

// NRF24L01命令
#define NRF24_CMD_R_REGISTER    0x00
#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_W_TX_PAYLOAD  0xA0
#define NRF24_CMD_FLUSH_TX      0xE1
#define NRF24_CMD_FLUSH_RX      0xE2
#define NRF24_CMD_REUSE_TX_PL   0xE3
#define NRF24_CMD_NOP           0xFF

// NRF24L01寄存器
#define NRF24_REG_CONFIG        0x00
#define NRF24_REG_EN_AA         0x01
#define NRF24_REG_EN_RXADDR     0x02
#define  NRF24_REG_SETUP_AW      0x03
#define NRF24_REG_SETUP_RETR    0x04
#define NRF24_REG_RF_CH         0x05
#define NRF24_REG_RF_SETUP      0x06
#define NRF24_REG_STATUS        0x07
#define NRF24_REG_OBSERVE_TX    0x08
#define NRF24_REG_CD            0x09
#define  NRF24_REG_RX_ADDR_P0    0x0A
#define NRF24_REG_RX_ADDR_P1    0x0B
#define NRF24_REG_TX_ADDR       0x10
#define NRF24_REG_RX_PW_P0      0x11
#define NRF24_REG_FIFO_STATUS   0x17

// 数据长度定义
#define NRF24_PAYLOAD_SIZE      3 //最大32
#define NRF24_ADDR_LEN          5

extern uint8_t nrf24_addr[5];

// 函数声明
void NRF24_Init(SPI_HandleTypeDef *hspi);
void NRF24_SetTxMode(uint8_t *tx_addr);
void NRF24_SetRxMode(uint8_t *rx_addr);
uint8_t NRF24_SendData(uint8_t *data, uint8_t len);
uint8_t NRF24_ReceiveData(uint8_t *data, uint8_t len);
uint8_t NRF24_Check(void);

#endif
