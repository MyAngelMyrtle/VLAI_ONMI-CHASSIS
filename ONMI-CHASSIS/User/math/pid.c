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
/**
  ******************************************************************************
  * @file			pid.c
  * @version		V1.1.0
  * @date			2024年4月30日03:04:08
  * @author          ZGH
  * @brief   		对于PID， 反馈/测量习惯性叫get/measure/real/fdb,
                          期望输入一般叫set/target/ref
  *******************************************************************************/
#define __PID_GLOBALS
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "mytype.h"
#include <math.h>
#include "data_processing.h"

// #include "cmsis_os.h"
enum
{
    LLAST = 0,
    LAST = 1,
    NOW = 2,
};
pid_t pid_chassis_spd[4];
#ifndef ABS
#define ABS(x) ((x > 0) ? (x) : (-x))
#endif

#define LIMIT(x,limit) (x)=(((x)<=(-limit))?(-limit):(((x)>=(limit))?(limit):(x)))

/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd,
    float deadband,
    float A,
    float B,
    uint8_t improve)
{

    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->deadband = deadband;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->CoefA = A;
    pid->CoefB = B;

    pid->Improve = improve;

}

/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
 * 计算差分PID和位置PID
 * 
 * 本函数用于根据目标值和测量值计算PID控制器的输出。它支持多种改进算法，包括梯形积分、变速积分、微分先行、微分滤波器和积分限幅。
 * 
 * @param[in] pid 指向pid_t结构体的指针，包含PID控制的参数和状态。
 * @param[in] get 当前测量值。
 * @param[in] set 目标值。
 * @return 计算得到的PID输出值。根据PID的工作模式，返回的位置PID输出或差分PID输出。
 */
float pid_calc(pid_t *pid, float get, float set)
{
    // 更新当前的测量值、目标值和误差
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // 计算误差

    // 检查误差是否超过最大误差限制，或是否处于死区范围内
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    // 计算PID各部分输出
    pid->pout = pid->p * pid->err[NOW]; // 比例项
    pid->ITerm =  pid->i *pid->err[NOW]; // 积分项（即时值）
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]); // 微分项

    // 应用各种改进算法
    // 梯形积分
    if (pid->Improve & Trapezoid_Intergral)
        f_Trapezoid_Intergral(pid);
    // 变速积分
    if (pid->Improve & ChangingIntegrationRate)
        f_Changing_Integration_Rate(pid);
    // 微分先行
    if (pid->Improve & Derivative_On_Measurement)
        f_Derivative_On_Measurement(pid);
    // 微分滤波器
    if (pid->Improve & DerivativeFilter)
        f_Derivative_Filter(pid);
    // 积分限幅
    if (pid->Improve & Integral_Limit)
        f_Integral_Limit(pid);
		
		//积分累加
		pid->iout += pid->ITerm;
		
    // 对积分输出进行限幅
    abs_limit(&(pid->iout), pid->IntegralLimit, 0);

    // 计算最终的PID输出
    pid->pos_out = pid->pout + pid->iout + pid->dout; // 计算位置PID输出

    // 对PID输出进行最大值限制
    abs_limit(&(pid->pos_out), pid->MaxOutput, 0);

    // 更新上次输出值
    pid->last_pos_out = pid->pos_out;
			
    // 更新误差和测量值的历史记录
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    // 根据PID模式返回相应的输出值
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

/*pid总体初始化-----------------------------------------------------------------*/
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
    uint8_t improve)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    /*init pid param */
    pid->f_param_init(pid,maxout, intergral_limit, kp, ki, kd,deadband,A,B,improve);
}

void abs_limit(float *a, float ABS_MAX, float offset)
{
    if (*a > ABS_MAX + offset)
        *a = ABS_MAX + offset;
    if (*a < -ABS_MAX + offset)
        *a = -ABS_MAX + offset;
}

/**********************************************************************************************************
 *函 数 名: FeedForward_Calc
 *功能说明: 前馈算法
 *形    参: PID_Struct *P  PID参数结构体
 *        ActualValue    PID计算反馈量（当前真实检测值）
 *返 回 值: PID反馈计算输出值
 **********************************************************************************************************/
void FeedForward_Calc(FeedForward_Typedef *FF, float Now_DeltIn)
{
    FF->Now_DeltIn = Now_DeltIn;
    FF->Out = FF->Now_DeltIn * FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn) * FF->K2;
    FF->Last_DeltIn = FF->Now_DeltIn;
    abs_limit(&(FF->Out), FF->OutMax, 0);
}

/**********************************************************************************************************
 *函 数 名: D_AGL_FeedForward_Calc
 *功能说明: 前馈算法(基于外环期望微分)
 *形    参: PID_Struct *P  PID参数结构体
 *        ActualValue    PID计算反馈量（当前真实检测值）
 *返 回 值: PID反馈计算输出值
 **********************************************************************************************************/
void D_AGL_FeedForward_Calc(D_AGL_FeedForward_Typedef *AGL_FF, float now_Ref, float period)
{
    AGL_FF->Now_ref = now_Ref;
    AGL_FF->slope = (AGL_FF->Now_ref - AGL_FF->Last_ref);
    AGL_FF->Out = AGL_FF->k * ((AGL_FF->Now_ref - AGL_FF->Last_ref) / period);
    AGL_FF->Last_ref = AGL_FF->Now_ref;
}

/*
 * 函数名称：f_Trapezoid_Intergral
 * 功能描述：计算并更新PID控制器的积分项。
 * 参数列表：
 *  - pid: 指向包含PID控制状态的结构体的指针。
 * 返回值：无。
 */

static void f_Trapezoid_Intergral(pid_t *pid)
{
    // 计算积分项，采用梯形法则进行积分
    pid->ITerm = pid->i * (( pid->err[NOW] + pid->err[LAST]) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(pid_t *pid)
{
    if (pid->err[NOW] * pid->iout > 0)
    {
        // 积分呈累积趋势
        // Integral still increasing
        if (ABS(pid->err[NOW]) <= pid->CoefB)
            return; // Full integral
        if (ABS(pid->err[NOW]) <= (pid->CoefA + pid->CoefB))//       B < err < A+B
            pid->ITerm *= (pid->CoefA - ABS(pid->err[NOW]) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;// cancel integral
    }
}

static void f_Integral_Limit(pid_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->iout + pid->ITerm;
    temp_Output = pid->pout + pid->iout + pid->dout;
    if (ABS(temp_Output) > pid->MaxOutput)
    {
        if (pid->err[NOW] * pid->iout > 0)
        {
            // 积分呈累积趋势
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

			if(ABS(temp_Iout) >ABS( pid->IntegralLimit))
				
				 pid->ITerm = 0;
			
		 LIMIT(temp_Iout,pid->IntegralLimit);
		
}


static void f_Derivative_On_Measurement(pid_t *pid)
{

        // if (pid->OLS_Order > 2)
        //     pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        // else
        //     pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    

}


static void f_Derivative_Filter(pid_t *pid)
{
    pid->dout = pid->dout * pid->dt / (pid->derivative_lpf_rc + pid->dt) +
                pid->last_delta_out * pid->derivative_lpf_rc / (pid->derivative_lpf_rc + pid->dt);
}

