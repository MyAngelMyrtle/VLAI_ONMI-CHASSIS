#ifndef __RobStride_H__
#define __RobStride_H__

#include "main.h"
#include "bsp_fdcan.h"

void RbsEnable(hcan_t *hfdcan, uint32_t id);
void RbsDisable(hcan_t *hfdcan, uint32_t id);
void RbsZeroPointSet(hcan_t *hfdcan, uint32_t id);
void RbsModeSet(hcan_t *hfdcan, uint32_t id, uint8_t mode);
void RbsProtocolSet(hcan_t *hfdcan, uint32_t id);
void RbsPosCtrl(hcan_t *hfdcan, uint32_t id, float rad, float rad_s);

#endif
