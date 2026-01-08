#include "bsp_fdcan.h"
#include "drv_dmmotor.h"
#include <string.h>
dmmoto_msg_t chassis_moto_data[4];

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void mit_ctrl(hcan_t *hfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
  static uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE; // MIT_MODE=0x00

  pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
  kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  fdcanx_send_data(&hfdcan1, id, data, 8);
}

void speed_ctrl(hcan_t *hfdcan, uint32_t motor_id, float spd)
{
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + 0x200;
  vbuf = (uint8_t *)&spd;
	memset(data, 0, sizeof(data));
  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  fdcanx_send_data(&hfdcan1, id, data, 4);
}

void DM_ENABLE(hcan_t *hfdcan, uint32_t id)
{
	uint32_t motor_id = id + 0x200;
  static uint8_t txdata[8];
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0XFF;
  txdata[7] = (int16_t)0XFC;
	
  fdcanx_send_data(&hfdcan1, motor_id, txdata, 8); // speed mode
}

void DM_DISABLE(hcan_t *hfdcan, uint32_t id)
{
  static uint8_t txdata[8];
	uint32_t motor_id = id + 0x200;
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0XFF;
  txdata[7] = (int16_t)0XFD;

  fdcanx_send_data(&hfdcan1, motor_id, txdata, 8);
}

void DM_ZERO_RESET(hcan_t *hfdcan, uint32_t id)
{
  static uint8_t txdata[8];
  txdata[0] = (int16_t)0XFF;
  txdata[1] = (int16_t)0XFF;
  txdata[2] = (int16_t)0XFF;
  txdata[3] = (int16_t)0XFF;
  txdata[4] = (int16_t)0XFF;
  txdata[5] = (int16_t)0XFF;
  txdata[6] = (int16_t)0XFF;
  txdata[7] = (int16_t)0XFE;

  fdcanx_send_data(&hfdcan1, id, txdata, 8);
}

void mf_DM_MOTOR_HANDLE(uint8_t *data, dmmoto_msg_t *dmmotor_data, uint32_t id)
{
  dmmotor_data->online = 1;
  dmmotor_data->err = (data[0] >> 4) & 0x0F;
  dmmotor_data->pos = (uint16_t)((uint16_t)data[1] << 8 | (uint16_t)data[2]);
  /* 12-bit speed: high 8 bits in D3 (VEL[11:4]), low 4 bits in D4[7:4] */
  uint16_t spd = (int16_t)((uint16_t)data[3] << 4 | (uint16_t)((data[4] >> 4) & 0x0F));
  /* sign-extend 12-bit to 16-bit */
  dmmotor_data->spd = uint_to_float(spd, V_MIN, V_MAX, 12);
  /* 12-bit torque: high 4 bits in D4[3:0], low 8 bits in D5 */
  dmmotor_data->torque = (uint16_t)(((uint16_t)(data[4] & 0x0F) << 8) | (uint16_t)data[5]);
  dmmotor_data->Temp = data[6];
  dmmotor_data->tmos_Temp = data[7];
}
