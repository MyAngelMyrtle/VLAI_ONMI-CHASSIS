#include "main.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "EZ_RTOS.h"
#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float gyro[3], accel[3], temp;
uint8_t forceStop = 0;

uint8_t imuBinarySem01Handle = 0;

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;
uint32_t task_time;
/**
************************************************************************
* @brief:      	IMU_TempCtrlTask(void const * argument)
* @param:       argument - 任务参数
* @retval:     	void
* @details:    	IMU温度控制任务函数
************************************************************************
**/
void IMU_TempCtrlTask(void)
{
//        if(imuBinarySem01Handle)
//				{
//				imuBinarySem01Handle = 0;
//				BMI088_init();
        BMI088_read(gyro, accel, &temp);
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        
        if (forceStop == 1)
        {
            out = 0.0f;
        }
        
        htim3.Instance->CCR4 = (uint16_t)out;
				task_time++;
//				task_delay(10);
//			}
}
/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
			imuBinarySem01Handle = 1;
//        osSemaphoreRelease(imuBinarySem01Handle);
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}
