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
    float x;
    float y;
		float z;
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
		
		uint16_t host_online;
		uint16_t host_can_flag_handle;
		uint16_t host_can_flag_handle_last;
} chassis_t;

typedef struct
{
    float vx;
    float vy;
    float vw;

    float scale_ch1;
    float scale_ch2;
}scale_t;

void omni_odom_update(const float ds[4],
                      float L,
                      float *x,
                      float *y,
                      float *theta);

/*******************OOP*********************************/
extern chassis_t chassis;
void chassis_task(void);
void chassis_init(void);
void chassis_spd_distribution(void);
void chassis_pid_calcu(void);
void Chassis_ctrl(void);
void mini_car_calc(float vy, float vw, float speed[]);
float data_limit(float data, float max, float min);
void Chassis_ctrl(void);
void chassis_odom_calc(void);
void odom_update(odom_t *odom, float dx_body, float dy_body, float dtheta);

/* 根据 wheel_pos_move（counts）计算本次角增量（弧度） */
float chassis_calc_dtheta_from_wheelpos(const int16_t wheel_pos_move[4], float L);
/* 预测使用 wheel_pos_move 更新后的航向角（弧度），不改变 chassis.odom.z */
float chassis_predict_yaw_from_wheelpos(const int16_t wheel_pos_move[4], float L);

#define PI_F        3.14159265l
#define SQRT2_F     1.41421356l
#define ENCODER_MAX 65535.0l
#define RAD_2_ANGEL 57.2957795l
#endif
