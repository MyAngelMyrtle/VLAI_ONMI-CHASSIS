#include "controler_task.h"
#include "chassis_task.h"
#include "EZ_RTOS.h"
#include "bsp_fdcan.h"
#include "uart_bsp.h"
#include "usart.h"
#include "gpio.h"
ctrl_mode_e ctrl_mode;

#define ARR1 20000
#define ARR2 20000

#define defaultCH 992
#define MaxCH 1792
#define MinCH 192

const float sc = 0.01f;

uint8_t ch_signal_effective[10];
  
void controler_task(void)
{
    remoteHandler();
		task_delay(5);
}

int abs(int x) {
    return x < 0 ? -x : x;
}

void remoteHandler(void)
{
		ctrl_mode =
    (remoter.rc.ch[7] < (defaultCH - 20)) ? PROTECT_MODE :
    (remoter.rc.ch[7] > (defaultCH + 20)) ? AUTO_MODE :
    REMOTER_MODE;
		if(remoter.rc.ch[7] == 0)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2);
			ctrl_mode = PROTECT_MODE;
		}
	if(ctrl_mode == REMOTER_MODE)
	{
	 ch_signal_effective[1] = (remoter.rc.ch[1] < (defaultCH - 20)||remoter.rc.ch[1] > (defaultCH + 20))?1:0;
	 ch_signal_effective[0] = (remoter.rc.ch[0] < (defaultCH - 20)||remoter.rc.ch[0] > (defaultCH + 20))?1:0;
	 ch_signal_effective[3] = (remoter.rc.ch[3] < (defaultCH - 20)||remoter.rc.ch[3] > (defaultCH + 20))?1:0;
	 
	 chassis.spd_input.vy = (float)((ch_signal_effective[1])?(remoter.rc.ch[1] - defaultCH)*sc:0);
	 chassis.spd_input.vx = (float)((ch_signal_effective[0])?(remoter.rc.ch[0] - defaultCH)*sc:0);
	 chassis.spd_input.vw = (float)((ch_signal_effective[3])?(remoter.rc.ch[3] - defaultCH)*sc:0);
	 
	 if(remoter.rc.ch[4] > (defaultCH + 200))
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	 else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	 if(remoter.rc.ch[4] < (defaultCH - 200))
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	 else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
	}
	else if(ctrl_mode == AUTO_MODE)
	{
		chassis.spd_input.vy = can_spd_input.vy*sc;
		chassis.spd_input.vx = can_spd_input.vx*sc;
		chassis.spd_input.vw = can_spd_input.vw*sc;
		
		if(can_spd_input.updown == 2)
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
		if(can_spd_input.updown == 1)
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
	}
}




