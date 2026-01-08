#ifndef __DRV_8030_H__
#define __DRV_8030_H__

#include "stm32h7xx_hal.h"
#include "bsp_fdcan.h"

typedef struct
{
	uint16_t speed;
}s8030_moto_t;

s8030_moto_t moto1;
s8030_moto_t moto2;

void MotoSetOperational(hcan_t *hfdcan);
void MotoSetSpeedMode(hcan_t *hfdcan, uint8_t node_id);
void MotoSetAcceleration(hcan_t *hfdcan, uint8_t node_id, uint32_t accel_time);
void MotoSetDeceleration(hcan_t *hfdcan, uint8_t node_id, uint32_t decel_time);
void MotoSetTargetSpeed(hcan_t *hfdcan, uint8_t node_id, int32_t target_speed);
void MotoControlWordFirstReady(hcan_t *hfdcan, uint8_t node_id);
void MotoControlWordDisable(hcan_t *hfdcan, uint8_t node_id);
void MotoControlWordEnable(hcan_t *hfdcan, uint8_t node_id);
void MotoConfigureTPDO1(hcan_t *hfdcan, uint8_t node_id);
void MotoConfigureTPDO2(hcan_t *hfdcan, uint8_t node_id);
void MotoSendSyncFrame(hcan_t *hfdcan);
void MotoSpeedConfigExample(hcan_t *hfdcan);


#endif
