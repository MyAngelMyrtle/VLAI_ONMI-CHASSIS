#include "chassis_task.h"
#include "bsp_fdcan.h"
#include "drv_dmmotor.h"
#include "EZ_RTOS.h"
#include "pid.h"
#include "controler_task.h"
// Simplified RTOS in C with task delay support
#include <stdint.h>
#include <stdio.h>
#include <string.h>

chassis_t chassis;
scale_t scale;

uint8_t disable_flag;

static Chassis_Derived Drv_PROTECT;
static Chassis_Derived Drv_REMOTER;
static Chassis_Derived Drv_AUTO;

static void Chassis_MODE_PROTECT_callback(void);
static void Chassis_MODE_AUTO_callback(void);
static void Chassis_MODE_REMOTER_callback(void);

static Chassis_Base *Chassis_mode_switch(void);

static Chassis_Base *Action_ptr = NULL;
void chassis_task(void) 
{
    Action_ptr = Chassis_mode_switch();

    Action_ptr->Control_Fun();

    /* 发送电流，控制电机 */
    Chassis_ctrl();
		task_delay(3);
}

void chassis_init(void)
{
	for(uint8_t i=0;i<4;i++)
	{
    chassis_moto_data[i].spd = 0;
	}
	
    for (uint8_t i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_chassis_spd[i], 7, 7, 0.7f,
                    0.00004f, 0.0f, 0, 0, 0, Integral_Limit);
  }
  scale.scale_ch1 = 0.004f;
  scale.scale_ch2 = 0.004f;

  Drv_PROTECT.Base.Control_Fun = Chassis_MODE_PROTECT_callback;
  Drv_REMOTER.Base.Control_Fun = Chassis_MODE_REMOTER_callback;
  Drv_AUTO.Base.Control_Fun = Chassis_MODE_AUTO_callback;
}

static Chassis_Base *Chassis_mode_switch(void)
{
  /* 系统历史状态机 */
  static Chassis_Base *p_return = NULL;
  static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;

  /* 底盘状态机 */
  switch (ctrl_mode)
  {
  case PROTECT_MODE: // 能量模式和保护模式下，底盘行为相同
  {
    p_return = (Chassis_Base *)&Drv_PROTECT;
    chassis.mode = CHASSIS_MODE_PROTECT;
  }
  break;
  case REMOTER_MODE:
  {
    if (last_ctrl_mode != REMOTER_MODE) // 切入遥控模式，初始化底盘模式
      chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
    /* 底盘小陀螺模式 */
    p_return = (Chassis_Base *)&Drv_REMOTER;
  }
  break;
  case AUTO_MODE:
  {
    chassis.mode = CHASSIS_MODE_AUTO;
    p_return = (Chassis_Base *)&Drv_AUTO;
  }
  break;
  default:
    break;
  }
  /* 系统历史状态更新 */
  last_ctrl_mode = ctrl_mode;
  return p_return;
}

static void Chassis_MODE_PROTECT_callback(void)
{
  chassis.spd_input.vx = 0;
  chassis.spd_input.vy = 0;
  chassis.spd_input.vw = 0;
//	if(disable_flag == 1)
//	{
//		task_delay(100);
//		DM_DISABLE(&hfdcan1,0x021);
//		DM_DISABLE(&hfdcan1,0x022);
//		DM_DISABLE(&hfdcan1,0x023);
//		DM_DISABLE(&hfdcan1,0x024);
//	}
	disable_flag = 0;
//  memset(&chassis.speed_send, 0, sizeof(chassis.speed_send)); 

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_chassis_spd[i].iout = 0;
  }
}

static void Chassis_MODE_REMOTER_callback(void)
{
  chassis.spd_input.vx = chassis.spd_input.vx;
  chassis.spd_input.vy = chassis.spd_input.vy;
  chassis.spd_input.vw = chassis.spd_input.vw;
//	if(disable_flag == 0)
//	{
//		task_delay(100);
//		DM_ENABLE(&hfdcan1,0x021);
//		DM_ENABLE(&hfdcan1,0x022);
//		DM_ENABLE(&hfdcan1,0x023);
//		DM_ENABLE(&hfdcan1,0x024);
//		disable_flag = 1;
//	}
  chassis_spd_distribution();
  chassis_pid_calcu();
}

static void Chassis_MODE_AUTO_callback(void)
{
  chassis.spd_input.vx = chassis.spd_input.vx;
  chassis.spd_input.vy = chassis.spd_input.vy;
  chassis.spd_input.vw = chassis.spd_input.vw;
//	if(disable_flag == 0)
//	{
//		task_delay(100);
//		DM_ENABLE(&hfdcan1,0x021);
//		DM_ENABLE(&hfdcan1,0x022);
//		DM_ENABLE(&hfdcan1,0x023);
//		DM_ENABLE(&hfdcan1,0x024);
//		disable_flag = 1;
//	}
  chassis_spd_distribution();
  chassis_pid_calcu(); // 舵轮pid计算
}

void chassis_spd_distribution(void)
{
  mecanum_calc(chassis.spd_input.vx, chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
  /* 速度重分配 并 限幅 */
  for (uint8_t j = 0; j < 4; j++)
  {
    chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j];
    chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j], 8.0f, -8.0f); // 电机转速最高到8900
  }

  chassis.odom.x += chassis.spd_fdb.vx * 0.001f;
  chassis.odom.y += chassis.spd_fdb.vy * 0.001f;
}

void chassis_pid_calcu(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis.wheel_spd_fdb[i] = chassis_moto_data[i].spd;
    chassis.speed_send[i] = pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
  }
}

void Chassis_ctrl(void)
{
	
	for (uint8_t i = 0; i < 4; i++)
	{
		if(ctrl_mode == PROTECT_MODE)
		speed_ctrl(&hfdcan1, 0x021+i, 0);
		else
		speed_ctrl(&hfdcan1, 0x021+i, chassis.speed_send[i]);
	}
	
}

void mecanum_calc(float vx, float vy, float vw, float speed[])
{
  float wheel_rpm[4];
  wheel_rpm[0] = -vx + vy + vw;
  wheel_rpm[1] = vx + vy + vw;
  wheel_rpm[2] = vx - vy + vw;
  wheel_rpm[3] = -vx - vy + vw;
  memcpy(speed, wheel_rpm, 4 * sizeof(float));
}

float data_limit(float data, float max, float min)
{
    if(data >= max)					return max;
    else if(data <= min)		return min;
    else 										return data;
}

