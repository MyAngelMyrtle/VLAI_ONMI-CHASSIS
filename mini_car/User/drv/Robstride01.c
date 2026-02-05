#include "RobStride.h"
#include "string.h"

void RbsEnable(hcan_t *hfdcan, uint32_t id)
{
  uint32_t motor_id = id;
  static uint8_t txdata[8];
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0XFF;
  txdata[7] = (int16_t)0XFC;

  fdcanx_send_data(&hfdcan2, motor_id, txdata, 8); // speed mode
}

void RbsDisable(hcan_t *hfdcan, uint32_t id)
{
  static uint8_t txdata[8];
  uint32_t motor_id = id;
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0XFF;
  txdata[7] = (int16_t)0XFD;

  fdcanx_send_data(&hfdcan2, motor_id, txdata, 8);
}

void RbsZeroPointSet(hcan_t *hfdcan, uint32_t id)
{
  static uint8_t txdata[8];
  uint32_t motor_id = id;
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0XFF;
  txdata[7] = (int16_t)0XFE;

  fdcanx_send_data(&hfdcan2, motor_id, txdata, 8);
}

void RbsModeSet(hcan_t *hfdcan, uint32_t id, uint8_t mode)
{
  static uint8_t txdata[8];
  uint32_t motor_id = id;
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)mode;
  txdata[7] = (int16_t)0XFC;

  fdcanx_send_data(&hfdcan2, motor_id, txdata, 8);
}

void RbsProtocolSet(hcan_t *hfdcan, uint32_t id)
{
  static uint8_t txdata[8];
  uint32_t motor_id = id;
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0X02;
  txdata[7] = (int16_t)0XFD;

  fdcanx_send_data(&hfdcan2, motor_id, txdata, 8);
}

void RbsPosCtrl(hcan_t *hfdcan, uint32_t id, float rad, float rad_s)
{
  static uint8_t txdata[8];
  uint32_t motor_id = id;
  
  // 将 float 类型转换为字节数组（小端格式）
  // 前4字节：rad_s (速度)
  memcpy(&txdata[0], &rad_s, 4);
  
  // 后4字节：rad (电流/位置)
  memcpy(&txdata[4], &rad, 4);

  fdcanx_send_data(hfdcan, motor_id, txdata, 8);
}


