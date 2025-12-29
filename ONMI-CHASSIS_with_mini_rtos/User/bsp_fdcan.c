#include "bsp_fdcan.h"
#include "drv_dmmotor.h"
#include "chassis_task.h"
FDCAN_RxHeaderTypeDef rx_fifo0_message, rx_fifo1_message;
uint8_t rx_fifo0_data[64], rx_fifo1_data[64];
can_spd_input_t can_spd_input;
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{
//	can_filter_init();
//	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
//	HAL_FDCAN_Start(&hfdcan2);
//	HAL_FDCAN_Start(&hfdcan3);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
	fdcan_filter.FilterID1 = 0x031;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
	fdcan_filter.FilterIndex = 1;
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
	fdcan_filter.FilterID1 = 0x032;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
	fdcan_filter.FilterIndex = 2;
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
	fdcan_filter.FilterID1 = 0x033;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
	fdcan_filter.FilterIndex = 3;
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
	fdcan_filter.FilterID1 = 0x034;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能邮箱0新消息中断
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 使能邮箱1新消息中断
	HAL_FDCAN_Start(&hfdcan1);
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
	fdcan_filter.FilterID1 = 0x050;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // 通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能邮箱0新消息中断
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 使能邮箱1新消息中断
	HAL_FDCAN_Start(&hfdcan2);
	
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_fifo0_message, rx_fifo0_data);
		if (hfdcan->Instance == FDCAN1)
		{
			switch (rx_fifo0_message.Identifier)
			{
			case (0x031):
			{
				mf_DM_MOTOR_HANDLE(rx_fifo0_data, &chassis_moto_data[0],rx_fifo0_message.Identifier);
				break;
			}
			case (0x032):
			{
				mf_DM_MOTOR_HANDLE(rx_fifo0_data, &chassis_moto_data[1],rx_fifo0_message.Identifier);
				break;
			}
			case (0x033):
			{
				mf_DM_MOTOR_HANDLE(rx_fifo0_data, &chassis_moto_data[2],rx_fifo0_message.Identifier);
				break;
			}
			case (0x034):
			{
				mf_DM_MOTOR_HANDLE(rx_fifo0_data, &chassis_moto_data[3],rx_fifo0_message.Identifier);
				break;
			}
			default:
				break;
			}
		}
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_fifo1_message, rx_fifo1_data);
		if (hfdcan->Instance == FDCAN2)
		{
			switch (rx_fifo1_message.Identifier)
			{
			case (0x050):
			{
				chassis_automode_msg_handle();
				break;
			}
			default:
				break;
			}
		}
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	}
}

int16_t float_to_int16_3dp(float x)
{
    return (int16_t)lroundf(x * 1000.0f);
}

void msg_to_Host(float odx,float ody,float odz)
{
	static int16_t odx_3p,ody_3p,odz_3p;
	odx_3p = float_to_int16_3dp(odx);
	ody_3p = float_to_int16_3dp(ody);
	odz_3p = float_to_int16_3dp(odz);
	
	static uint8_t txdata[8] ;
	txdata[0] = (int16_t)odx_3p >> 8;
	txdata[1] = (int16_t)odx_3p;
	txdata[2] = (int16_t)ody_3p >> 8;
	txdata[3] = (int16_t)ody_3p;
	txdata[4] = (int16_t)odz_3p >> 8;
	txdata[5] = (int16_t)odz_3p;
	
	fdcanx_send_data(&hfdcan2, 0x051, txdata, 8); // speed mode
}

void chassis_automode_msg_handle(void)
{
	can_spd_input.vx =(int16_t)(rx_fifo1_data[1]<<8|rx_fifo1_data[0]);
	can_spd_input.vy =(int16_t)(rx_fifo1_data[3]<<8|rx_fifo1_data[2]);
	can_spd_input.vw =(int16_t)(rx_fifo1_data[5]<<8|rx_fifo1_data[4]);
}
