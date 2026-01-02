#include "nrf24.h"

static SPI_HandleTypeDef *nrf24_hspi;


uint8_t nrf24_addr[5] = {0x34, 0x43, 0x10, 0x10, 0x01};

// 片选控制
static void NRF24_CS_LOW(void) {
  HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);
}

static void NRF24_CS_HIGH(void) {
  HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

// 芯片使能控制
static void NRF24_CE_LOW(void) {
  HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

static void NRF24_CE_HIGH(void) {
  HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

// 向NRF24L01写入一个字节
static uint8_t NRF24_WriteByte(uint8_t data) {
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(nrf24_hspi, &data, &rx_data, 1, 100);
  return rx_data;
}

// 读取NRF24L01寄存器
uint8_t NRF24_ReadReg(uint8_t reg) {
  uint8_t cmd, data;

  NRF24_CS_LOW();
  cmd = NRF24_CMD_R_REGISTER | reg;
  NRF24_WriteByte(cmd);
  data = NRF24_WriteByte(NRF24_CMD_NOP);
  NRF24_CS_HIGH();

  return data;
}

// 写入NRF24L01寄存器
void NRF24_WriteReg(uint8_t reg, uint8_t data) {
  uint8_t cmd;

  NRF24_CS_LOW();
  cmd = NRF24_CMD_W_REGISTER | reg;
  NRF24_WriteByte(cmd);
  NRF24_WriteByte(data);
  NRF24_CS_HIGH();
}

// 写入多字节寄存器
void NRF24_WriteRegMulti(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t cmd;
  uint8_t i;

  NRF24_CS_LOW();
  cmd = NRF24_CMD_W_REGISTER | reg;
  NRF24_WriteByte(cmd);

  for (i = 0; i < len; i++) {
    NRF24_WriteByte(data[i]);
  }

  NRF24_CS_HIGH();
}

// 读取多字节寄存器
void NRF24_ReadRegMulti(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t cmd;
  uint8_t i;

  NRF24_CS_LOW();
  cmd = NRF24_CMD_R_REGISTER | reg;
  NRF24_WriteByte(cmd);

  for (i = 0; i < len; i++) {
    data[i] = NRF24_WriteByte(NRF24_CMD_NOP);
  }

  NRF24_CS_HIGH();
}

// 初始化NRF24L01
void NRF24_Init(SPI_HandleTypeDef *hspi) {
  nrf24_hspi = hspi;

  // 初始化CE和CS引脚为输出
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = NRF24_CE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF24_CE_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF24_CS_PIN;
  HAL_GPIO_Init(NRF24_CS_PORT, &GPIO_InitStruct);

  // 初始状态
  NRF24_CE_LOW();
  NRF24_CS_HIGH();

  // 等待芯片就绪
  HAL_Delay(100);

  // 复位
  NRF24_WriteReg(NRF24_REG_CONFIG, 0x00);

  // 使能自动应答
  NRF24_WriteReg(NRF24_REG_EN_AA, 0x01); // 仅P0通道使能自动应答

  // 使能接收通道
  NRF24_WriteReg(NRF24_REG_EN_RXADDR, 0x01); // 仅使能P0通道

  // 设置地址宽度为5字节
  NRF24_WriteReg(NRF24_REG_SETUP_AW, 0x03); // 5字节地址

  // 设置自动重发，等待250us，重发3次
  NRF24_WriteReg(NRF24_REG_SETUP_RETR, 0x23);

  // 设置RF通道为40
  NRF24_WriteReg(NRF24_REG_RF_CH, 0x20);

  // 设置数据传输率为2Mbps，发射功率0dBm
  NRF24_WriteReg(NRF24_REG_RF_SETUP, 0x0F);
  // NRF24_WriteReg(NRF24_REG_RF_SETUP, 0x07); // 1Mbps, 0dBm

  // 设置接收数据长度
  NRF24_WriteReg(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE);

  // 清除状态寄存器
  NRF24_WriteReg(NRF24_REG_STATUS, 0x70);

  // 进入待机模式
  NRF24_CE_LOW();
}

// 设置为发送模式
void NRF24_SetTxMode(uint8_t *tx_addr) {
  NRF24_CE_LOW();

  // 设置发送地址
  NRF24_WriteRegMulti(NRF24_REG_TX_ADDR, tx_addr, NRF24_ADDR_LEN);

  // 设置接收地址(用于自动应答)
  NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, tx_addr, NRF24_ADDR_LEN);

  // 配置为发送模式，上电
  NRF24_WriteReg(NRF24_REG_CONFIG, 0x0E);

  HAL_Delay(1);
  NRF24_CE_HIGH();
  HAL_Delay(1);
}

// 设置为接收模式
void NRF24_SetRxMode(uint8_t *rx_addr) {
  NRF24_CE_LOW();

  // 设置接收地址
  NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, rx_addr, NRF24_ADDR_LEN);

  // 配置为接收模式，上电
  NRF24_WriteReg(NRF24_REG_CONFIG, 0x0F);

  HAL_Delay(1);
  NRF24_CE_HIGH();
  HAL_Delay(1);
}

// 发送数据
uint8_t NRF24_SendData(uint8_t *data, uint8_t len) {
  uint8_t status;
  uint8_t i;

  if (len > NRF24_PAYLOAD_SIZE)
    len = NRF24_PAYLOAD_SIZE;

  NRF24_CE_LOW();

  // 写入发送数据
  NRF24_CS_LOW();
  NRF24_WriteByte(NRF24_CMD_W_TX_PAYLOAD);

  for (i = 0; i < len; i++) {
    NRF24_WriteByte(data[i]);
  }

  // 不足32字节的部分补0
  for (i = len; i < NRF24_PAYLOAD_SIZE; i++) {
    NRF24_WriteByte(0x00);
  }

  NRF24_CS_HIGH();

  // 启动发送
  NRF24_CE_HIGH();
  HAL_Delay(1);

  // 等待发送完成
  uint32_t timeout = 0;
  while (1) {
    status = NRF24_ReadReg(NRF24_REG_STATUS);

    if (status & (1 << 5)) // 发送成功
    {
      NRF24_WriteReg(NRF24_REG_STATUS, 1 << 5); // 清除发送完成标志
      NRF24_WriteReg(NRF24_CMD_FLUSH_TX, 0x00); // 清空发送FIFO
      return 0;
    }

    if (status & (1 << 4)) // 发送失败，达到最大重发次数
    {
      NRF24_WriteReg(NRF24_REG_STATUS, 1 << 4); // 清除最大重发标志
      NRF24_WriteReg(NRF24_CMD_FLUSH_TX, 0x00); // 清空发送FIFO
      return 1;
    }

    HAL_Delay(1);
    timeout++;
    if (timeout > 100) // 超时
    {
      return 2;
    }
  }
}

// 接收数据
uint8_t NRF24_ReceiveData(uint8_t *data, uint8_t len) {
  uint8_t status;
  uint8_t i;

  status = NRF24_ReadReg(NRF24_REG_STATUS);

  if (status & (1 << 6)) // 有数据接收
  {
    if (len > NRF24_PAYLOAD_SIZE)
      len = NRF24_PAYLOAD_SIZE;

    // 读取接收数据
    NRF24_CS_LOW();
    NRF24_WriteByte(NRF24_CMD_R_RX_PAYLOAD);

    for (i = 0; i < len; i++) {
      data[i] = NRF24_WriteByte(NRF24_CMD_NOP);
    }

    NRF24_CS_HIGH();

    // 清除接收完成标志
    NRF24_WriteReg(NRF24_REG_STATUS, 1 << 6);

    // 清空接收FIFO
    NRF24_WriteReg(NRF24_CMD_FLUSH_RX, 0x00);

    return len;
  }

  return 0; // 没有接收到数据
}

// 检测NRF24L01是否存在
uint8_t NRF24_Check(void) {
  uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
  uint8_t rx_buf[5];

  // 写入测试地址
  NRF24_WriteRegMulti(NRF24_REG_TX_ADDR, buf, 5);
  // 读取地址
  NRF24_ReadRegMulti(NRF24_REG_TX_ADDR, rx_buf, 5);

  // 比较写入和读出的数据是否一致
  if (memcmp(buf, rx_buf, 5) == 0)
    return 1; // 检测成功
  else
    return 0; // 检测失败
}
