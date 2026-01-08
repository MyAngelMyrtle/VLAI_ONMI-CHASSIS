#ifndef __BSP_DMMOTOR_H
#define __BSP_DMMOTOR_H

#include "stm32h7xx_hal.h"
#include "bsp_fdcan.h"
typedef struct
{
	float spd_send;

	uint8_t err;
	uint16_t pos;
	uint16_t last_pos;
	float spd;
	uint8_t torque;
	uint8_t tmos_Temp;
	uint8_t Temp;
	uint8_t online;
} dmmoto_msg_t;

#define P_MIN -12.5F
#define P_MAX 12.5F
#define V_MIN -30.0F
#define V_MAX 30.0F
#define KP_MIN 0
#define KP_MAX 500.0
#define KD_MIN 0
#define KD_MAX 5.0
#define T_MIN -10.0F
#define T_MAX 10.0F
#define MIT_MODE 0x00

extern dmmoto_msg_t chassis_moto_data[4];

void DM_ENABLE(hcan_t *hfdcan, uint32_t id);
void mit_ctrl(hcan_t *hfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void DM_MOTOR_HANDLE(FDCAN_HandleTypeDef *hfdcan, uint8_t *data);
void speed_ctrl(hcan_t *hfdcan, uint32_t motor_id, float spd);
void DM_MOTOR_HANDLE_CHASSIS(FDCAN_HandleTypeDef *hfdcan, uint8_t *data);
void mf_DM_MOTOR_HANDLE(uint8_t *data, dmmoto_msg_t *dmmotor_data, uint32_t id);
void DM_ZERO_RESET(hcan_t *hfdcan, uint32_t id);
void DM_DISABLE(hcan_t *hfdcan, uint32_t id);

#endif
