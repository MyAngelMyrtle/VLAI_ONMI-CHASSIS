#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

void bsp_can_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);
void chassis_automode_msg_handle(void);
void msg_to_Host(float odx,float ody,float odz);

typedef struct
{
	float vy;
	float vx;
	float vw;
	uint16_t rc_flag;
}can_spd_input_t;

extern can_spd_input_t can_spd_input;

#endif /* __BSP_FDCAN_H_ */

