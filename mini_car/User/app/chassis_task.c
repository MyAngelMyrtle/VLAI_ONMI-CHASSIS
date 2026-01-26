#include "chassis_task.h"
#include "bsp_fdcan.h"
#include "drv_dmmotor.h"
#include "EZ_RTOS.h"
#include "pid.h"
#include "controler_task.h"
#include "drv_8030.h"
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

uint32_t running_time;

void chassis_task(void)
{
  Action_ptr = Chassis_mode_switch();

  Action_ptr->Control_Fun();

  /* 发送电流，控制电机 */
  Chassis_ctrl();
  chassis_odom_calc();
	msg_to_Host(chassis.odom.x,chassis.odom.y,chassis.odom.z);
  task_delay(3);
}

void chassis_init(void)
{
  for (uint8_t i = 0; i < 2; i++)
  {
    chassis_moto_data[i].spd = 0;
  }

  for (uint8_t i = 0; i < 2; i++)
  {
    PID_struct_init(&pid_chassis_spd[i], 200,80, 5.0f,
                    0.00f, 0.0f, 0.0f, 0, 0, Integral_Limit);
  }

  Drv_PROTECT.Base.Control_Fun = Chassis_MODE_PROTECT_callback;
  Drv_REMOTER.Base.Control_Fun = Chassis_MODE_REMOTER_callback;
  Drv_AUTO.Base.Control_Fun 	 = Chassis_MODE_AUTO_callback;
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

  disable_flag = 0;

  MotoSendSyncFrame(&hfdcan1);

  for (uint8_t i = 0; i < 2; i++)
  {
    pid_chassis_spd[i].iout = 0;
  }
}

static void Chassis_MODE_REMOTER_callback(void)
{
  chassis.spd_input.vx = chassis.spd_input.vx;
  chassis.spd_input.vy = chassis.spd_input.vy;
  chassis.spd_input.vw = chassis.spd_input.vw;

  disable_flag = 1;

  chassis_spd_distribution();
  chassis_pid_calcu();
}

static void Chassis_MODE_AUTO_callback(void)
{
	if(chassis.host_online > 5000)
	{
		chassis.spd_input.vx = 0;
		chassis.spd_input.vy = 0;
		chassis.spd_input.vw = 0;
	}
	else
	{
		chassis.spd_input.vx = chassis.spd_input.vx;
		chassis.spd_input.vy = chassis.spd_input.vy;
		chassis.spd_input.vw = chassis.spd_input.vw;
	}
	/* 构造 8 字节数据：按照 chassis_automode_msg_handle 的解析方式（大端 int16）组织 */
	uint8_t tx_data[8];
	/* 将当前期望速度打包为 int16（若需更高精度可乘以比例因子后再转换） */

	fdcanx_send_data(&hfdcan1, 0x050, tx_data, 8);
	
	chassis.host_can_flag_handle = can_spd_input.rc_flag;
	
	if(chassis.host_can_flag_handle != chassis.host_can_flag_handle_last)
		chassis.host_online = 0;
	else if(chassis.host_online < 5000)
		chassis.host_online++;
		
  chassis_spd_distribution();
  chassis_pid_calcu(); // 舵轮pid计算
	chassis.host_can_flag_handle_last = chassis.host_can_flag_handle;
}
int8_t spd_way[2] = {1,1};
void chassis_spd_distribution(void)
{
  mini_car_calc(chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
  /* 速度重分配 并 限幅 */
    chassis.wheel_spd_ref[0] = 1.5f*chassis.wheel_spd_input[0];
		chassis.wheel_spd_ref[1] = -1.5f*chassis.wheel_spd_input[1];
//    chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j], 100.8f, -100.8f); // 电机转速最高到8900
}

void chassis_pid_calcu(void)
{
  for (uint8_t i = 0; i < 2; i++)
  {
    chassis.wheel_spd_fdb[i] = moto[i].speed;
//    chassis.speed_send[i] = pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
		chassis.speed_send[i] = chassis.wheel_spd_ref[i];
  }
}

int16_t my_speed[2];

void Chassis_ctrl(void)
{
  for (uint8_t i = 0; i < 2; i++)
  {
		my_speed[i] = (int16_t)chassis.speed_send[i];
    if (ctrl_mode == PROTECT_MODE)
		{
      MotoSetSpeed(&hfdcan1, 0x01,0);
			MotoSetSpeed(&hfdcan1, 0x03,0);
		}
    else
		{
			if(i == 0)
			MotoSetSpeed(&hfdcan1, 0x01 ,my_speed[0]);
			else if(i == 1)
			MotoSetSpeed(&hfdcan1, 0x03 ,my_speed[1]);
		}
  }
//  MotoSpeedRead(&hfdcan1, 0x03);
//  MotoSpeedRead(&hfdcan1, 0x01);
}

uint8_t speed_pos = 4;

void mini_car_calc(float vy, float vw, float speed[])
{
  float wheel_rpm[2];
  wheel_rpm[0] =(vy + vw)*speed_pos;
  wheel_rpm[1] = (vy - vw)*speed_pos;
  memcpy(speed, wheel_rpm, 2 * sizeof(float));
}

float data_limit(float data, float max, float min)
{
  if (data >= max)
    return max;
  else if (data <= min)
    return min;
  else
    return data;
}

const double ratio = 0.00006806784l;
float single_wheel_move[2];
float last_single_wheel_move[2];
float wheel_pos_current[2];
float wheel_pos_last[2];
float wheel_pos_move[2];
uint32_t last_odom_ms = 0;
uint32_t now_ms = 0;
uint16_t times[2] = {0,0};
	uint8_t start;
void chassis_odom_calc(void)
{
  /* 基于时间的里程计采样，优先使用系统节拍（HAL_GetTick）而不是计数器内插 */
  static uint8_t last_all_online = 0;
  uint8_t online_cnt = 0;
	
  float ds[2];
	ds[0] = 0.0f;
  ds[1] = 0.0f;
  for (uint8_t i = 0; i < 2; i++)
  {
    if (moto[i].online)
      online_cnt++;
  }

  if (online_cnt == 2 && last_all_online == 0)
  {
    for (uint8_t i = 0; i < 2; i++)
    {
      wheel_pos_last[i] = 0.0l;
//      single_wheel_move[i] = 0.0l;
      last_single_wheel_move[i] = 0.0l;
    }
    last_odom_ms = HAL_GetTick();
  }
  last_all_online = (online_cnt == 2);
	
  now_ms = HAL_GetTick();
  const uint32_t ODOM_INTERVAL_MS = 5; /* 采样间隔（ms），可根据需要调整） */

  if (last_all_online && (now_ms - last_odom_ms >= ODOM_INTERVAL_MS))
  {
    float L = 0.375f; /* 底盘几何参数 */

    for (uint8_t i = 0; i < 2; i++)
    {
      /* 计数增量转换为线位移（meters） */
				
				ds[i] = (float)moto[i].speed * (float)ratio;
				single_wheel_move[i] += ds[i];
				last_single_wheel_move[i] = single_wheel_move[i];
				wheel_pos_last[i] = wheel_pos_current[i];
    }
		
		float v_linear = (ds[0] + ds[1]) / 2.0f;   // 底盘线速度
    float v_angular = (ds[1] - ds[0]) / L * 0.5f;      // 底盘角速度
		
    float dx_body = 0;
		float dy_body = v_linear;
    float dtheta  = v_angular;
		
    while (chassis.odom.z >= PI)
    {
        chassis.odom.z -= 2.0f*PI;
    }
    while (chassis.odom.z < -PI)
    {
        chassis.odom.z += 2.0f*PI;
    }
		
    odom_update(&chassis.odom,dx_body,dy_body,dtheta);

    last_odom_ms = now_ms;
  }
}
float angle;
void odom_update(odom_t *odom, float dx_body, float dy_body, float dtheta)
{
    // 1. 累计角
    odom->z -= dtheta;
		angle = odom->z *(180.0f/PI);
    // 2. 使用旋转增量公式 θ + dθ/2，提高大步长旋转精度
    float theta_mid = odom->z - dtheta / 2.0f;

    float c = cosf(theta_mid);  // 转成数学逆时针方向
    float s = sinf(theta_mid);

    // 3. 底盘坐标系 → 世界坐标系
    float dx_world = c * dx_body + s * dy_body;
    float dy_world = -s * dx_body + c * dy_body;

    // 4. 累加到世界坐标
    odom->x += dx_world;//*0.943f;
    odom->y += dy_world;//*0.943f;
}
