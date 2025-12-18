#include "controler_task.h"
#include "chassis_task.h"
#include "tim.h"
#include "EZ_RTOS.h"
#include "bsp_fdcan.h"
ctrl_mode_e ctrl_mode;

IC_t ic2_ch1 = {0};
IC_t ic2_ch3 = {0};
IC_t ic1_ch1 = {0};
IC_t ic1_ch3 = {0};

#define ARR1 20000
#define ARR2 20000

#define defaultCH 655

uint16_t TIM1_Detecter_Buf[2];
uint16_t TIM2_Detecter_Buf[1];
  
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
	if(ctrl_mode != AUTO_MODE)
	{
	 ic2_ch1.signal_effective = (ic2_ch1.high < 645||ic2_ch1.high > 655)?1:0;
	 ic1_ch1.signal_effective = (ic1_ch1.high < 645||ic1_ch1.high > 655)?1:0;
	 ic2_ch3.signal_effective = (ic2_ch3.high < 645||ic2_ch3.high > 655)?1:0;
	 
	 chassis.spd_input.vy = (ic2_ch1.signal_effective)?(ic2_ch1.high - CH_ORIGINAL_VALUE)*0.06f:0;
	 chassis.spd_input.vx = (ic1_ch1.signal_effective)?(ic1_ch1.high - CH_ORIGINAL_VALUE)*0.06f:0;
	 chassis.spd_input.vw = (ic2_ch3.signal_effective)?(ic2_ch3.high - CH_ORIGINAL_VALUE)*0.06f:0;
	 
    // 根据拨码开关(swd)的位置设置控制模式
    // 当swd小于0时为保护模式，等于0时为遥控模式，大于0时为自动模式
//    ctrl_mode =
//    (ic1_ch3.high < 645) ? PROTECT_MODE :
//    (ic1_ch3.high > 655) ? AUTO_MODE :
//    REMOTER_MODE;
	}
	else
	{
		chassis.spd_input.vy = can_spd_input.vy*0.008f;
		chassis.spd_input.vx = can_spd_input.vx*0.008f;
		chassis.spd_input.vw = can_spd_input.vw*0.008f;
		
//		ctrl_mode =
//    (ic1_ch3.high < 645) ? PROTECT_MODE :
//    (ic1_ch3.high > 655) ? AUTO_MODE :
//    REMOTER_MODE;
	}
}

static inline uint32_t diff(uint32_t now, uint32_t last, uint32_t arr)
{
    return (now >= last) ? (now - last) : (now + (arr + 1 - last));
}

/********************* 输入捕获回调 *********************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* ---------------- TIM2 CH1 ---------------- */
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (ic2_ch1.state == 0)   // 等上升沿
        {
            ic2_ch1.rise = val;
            ic2_ch1.state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else                     // 下降沿 → 计算高电平
        {
            ic2_ch1.high = diff(val, ic2_ch1.rise, ARR2);
            ic2_ch1.state = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
		/* ---------------- TIM2 CH3 ---------------- */
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

        if (ic2_ch3.state == 0)   // 等上升沿
        {
            ic2_ch3.rise = val;
            ic2_ch3.state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else                     // 下降沿 → 计算高电平
        {
            ic2_ch3.high = diff(val, ic2_ch3.rise, ARR2);
            ic2_ch3.state = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    /* ---------------- TIM1 CH1 ---------------- */
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (ic1_ch1.state == 0)
        {
            ic1_ch1.rise = val;
            ic1_ch1.state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic1_ch1.high = diff(val, ic1_ch1.rise, ARR1);
            ic1_ch1.state = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    /* ---------------- TIM1 CH3 ---------------- */
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

        if (ic1_ch3.state == 0)
        {
            ic1_ch3.rise = val;
            ic1_ch3.state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic1_ch3.high = diff(val, ic1_ch3.rise, ARR1);
            ic1_ch3.state = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}
