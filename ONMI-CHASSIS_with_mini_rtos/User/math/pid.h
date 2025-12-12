/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/
#ifndef __pid_H
#define __pid_H

#ifdef __PID_GLOBALS
#define __PID_EXT
#else
#define __PID_EXT extern
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

typedef enum
{
    POSITION_PID,
    DELTA_PID,
} PID_mode_e;

typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegrationRate = 0x20,     //0010 0000
    DerivativeFilter = 0x40,            //0100 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3]; // 目标值,包含NOW， LAST， LLAST上上次
    float get[3]; // 测量值
    float err[3]; // 误差

    float d_error;
    float pout; // p输出
    float iout; // i输出
    float dout; // d输出
    float ITerm;

    float pos_out;      // 本次位置式输出
    float last_pos_out; // 上次输出
    float delta_u;      // 本次增量值
    float delta_out;    // 本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;

    float CoefA; // For Changing Integral
    float CoefB; // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B

    float max_err;
    float deadband; // err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;     // 输出限幅
    uint32_t IntegralLimit; // 积分限幅

    uint8_t Improve;

    uint32_t DWT_CNT;
    float dt;

    float derivative_lpf_rc;

    void (*f_param_init)(struct __pid_t *pid, // PID参数初始化
                         uint32_t maxout,
                         uint32_t intergral_limit,
                         float kp,
                         float ki,
                         float kd,
                         float deadband,
                         float A,
                         float B,
                         uint8_t improve);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d); // pid三个参数修改

} pid_t;

void PID_struct_init(
    pid_t *pid,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd,
    float deadband,
    float A,
    float B,
    uint8_t improve);

typedef struct
{
    float K1;
    float K2;
    float Last_DeltIn;
    float Now_DeltIn;
    float Out;
    float OutMax;
} FeedForward_Typedef;

typedef struct
{
    float k;
    float Last_ref;
    float Now_ref;
    float Out;
    float OutMax;
    float slope;
} D_AGL_FeedForward_Typedef;

float pid_calc(pid_t *pid, float fdb, float ref);
void abs_limit(float *a, float ABS_MAX, float offset);
void D_AGL_FeedForward_Calc(D_AGL_FeedForward_Typedef *AGL_FF, float now_Ref, float period);
void FeedForward_Calc(FeedForward_Typedef *FF, float Now_DeltIn);
// PID优化环节函数声明
static void f_Trapezoid_Intergral(pid_t *pid);
static void f_Integral_Limit(pid_t *pid);
static void f_Derivative_On_Measurement(pid_t *pid);
static void f_Changing_Integration_Rate(pid_t *pid);
static void f_Derivative_Filter(pid_t *pid);
/* -------------------------------------- Gimbal -------------------------------------- */
// PIT 轴 位置速度串级
__PID_EXT pid_t pid_pit_ecd;
__PID_EXT pid_t pid_pit_angle;
__PID_EXT pid_t pid_pit_spd;
// YAW 轴 角度速度串级
__PID_EXT pid_t pid_yaw_angle_6020;
__PID_EXT pid_t pid_yaw_ecd_6020;
__PID_EXT pid_t pid_yaw_spd_6020;

__PID_EXT pid_t pid_yaw_angle_9025;
__PID_EXT pid_t pid_yaw_spd_9025;
__PID_EXT pid_t pid_yaw_angle_chassis;
__PID_EXT pid_t pid_yaw_spd_chassis;

__PID_EXT pid_t pid_9025_i;
// 测试YAW轴 位置速度串级 使用YAW电机反馈
__PID_EXT pid_t pid_yaw_mecd;
__PID_EXT pid_t pid_yaw_mspd;
/* -------------------------------------- Trigger -------------------------------------- */
__PID_EXT pid_t pid_trigger_hz;
__PID_EXT pid_t pid_trigger_ecd;
__PID_EXT pid_t pid_trigger_spd;

/* -------------------------------------- Vision -------------------------------------- */
__PID_EXT pid_t pid_vision_yaw;
__PID_EXT pid_t pid_vision_pit;

__PID_EXT pid_t pid_pitch_energy;
__PID_EXT pid_t pid_yaw_energy;
__PID_EXT pid_t pid_shoot_speed;
/* -------------------------------------- Chassis -------------------------------------- */
extern pid_t pid_chassis_spd[4];
__PID_EXT pid_t pid_chassis_angle; // 利用云台电机的编码器来实现底盘的跟随摇摆等控制
__PID_EXT pid_t pid_chassis_cur[4];

extern pid_t pid_chassis_6020_ecd[4];
extern pid_t pid_chassis_6020_spd[4];
extern pid_t pid_chassis_3508_spd[4];

#endif
