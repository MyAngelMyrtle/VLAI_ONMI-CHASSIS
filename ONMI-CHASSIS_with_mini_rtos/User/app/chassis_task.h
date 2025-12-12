/**
  * @file     chassis_task.h
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief
  *
  *	@author   Fatmouse
  *
  */
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "stm32h7xx_hal.h"

typedef void (*chassis_mode_callback)();

//父类
typedef struct
{
    chassis_mode_callback Control_Fun;
}Chassis_Base;

//子类
typedef struct 
{
    Chassis_Base Base;
}Chassis_Derived;

typedef enum 
{
    CHASSIS_MODE_PROTECT,
    CHASSIS_MODE_REMOTER_FOLLOW,
    CHASSIS_MODE_AUTO,  //自动步兵的自动模式  
    CHASSIS_MODE_REMOTER_ROTATE,
    CHASSIS_MODE_KEYBOARD_FOLLOW,
    CHASSIS_MODE_KEYBOARD_ROTATE,
    CHASSIS_MODE_KEYBOARD_FIGHT,
    CHASSIS_MODE_KEYBOARD_SUPPLY,
    
} chassis_mode_e;

// 底盘模式回调函数,用于解析协议
typedef void (*Chassis_mode_callback)();

typedef enum 
{
    ChasisInstance_MODE_PROTECT,
    ChasisInstance_MODE_REMOTER_FOLLOW_ROTATE, 
} ChasisInstance_mode_e;

typedef struct
{
	ChasisInstance_mode_e Chassis_Mode;
  Chassis_mode_callback mode_callback; // 解析收到的数据的回调函数
}ChasisInstance_t;

/* chassis parameter structure */
typedef struct
{
    float vx;
    float vy;
    float vw;
} spd_t;

typedef struct
{
    /* data */
    float  x;
    float y;
}odom_t;

typedef struct
{   
  chassis_mode_e mode;
    spd_t     spd_ref;
    spd_t     spd_input;
    spd_t     spd_fdb;
    odom_t    odom;
    float   wheel_spd_ref[4];
    float   wheel_spd_input[4];
    float   wheel_spd_fdb[4];
		
		int16_t   _2wSpeed[2];
		int16_t   _2wheel_spd_ref[2];
    int16_t   _2wheel_spd_input[2];
    int16_t   _2wheel_spd_fdb[2];
		
		float speed_send[4];
    int16_t   current_send[4];
		int16_t   rudder_angle_ref[4];
    int16_t   spd_error;
    int16_t   position_ref;
} chassis_t;

typedef struct
{
    float vx;
    float vy;
    float vw;

    float scale_ch1;
    float scale_ch2;
}scale_t;


/*******************OOP*********************************/
extern chassis_t chassis;
void chassis_task(void);
void chassis_init(void);
void chassis_spd_distribution(void);
void chassis_pid_calcu(void);
void Chassis_ctrl(void);
void mecanum_calc(float vx, float vy, float vw, float speed[]);
float data_limit(float data, float max, float min);
void Chassis_ctrl(void);

#endif
