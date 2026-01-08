#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "math.h"
#include "stm32h7xx_hal.h"

#define N2 100
#define e  2.718282f
#define SIGMOID_PERIOD 0.133333f
#define SIGMOID_MAX    10

#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII       3.14159f
#define MY_PPPIII_HALF  1.570796f
#define NATURAL_NUMBER  2.718281828f

#define pi 3.14159265358979323846f

#define DEG2RAD(angle) ((angle) * PI / 180.0f)


/*输出限幅*/
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

/*绝对值*/
#ifndef ABS
#define ABS(x)		((x>0)? (x): (-x))
#endif
typedef struct
{
    float change_scale;
    uint16_t real_target;
    uint16_t limit_target;
    TickType_t ticks;
    TickType_t last_ticks;
} Slope_Struct;

typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

void Slope_On(Slope_Struct *V);
typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

typedef struct {
    float cutoff_freq1;
    float b01, b11, b21, a11, a21;
    float delay_element_11, delay_element_21;
} Lpf2p;

extern ramp_function_source_t mwheel_ramp[4];
extern ramp_function_source_t chassis_w_ramp;
extern ramp_function_source_t chassis_super_x_ramp;
extern ramp_function_source_t chassis_super_y_ramp;

void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
//void Bubble_Sort(float *a,uint8_t n);
float GildeAverageValueFilter(float NewValue,float *Data);
float Sigmoid_function(float x);
float circle_error(float set,float get,float circle_para);
float data_limit(float data, float max, float min);
float lowpassfilter(float x);
//void medianFilter(float* signal, float* result, uint8_t N);
//int compare(const void *a, const void *b);
void Lpf2p_SetCutoffFreq(Lpf2p *lpf, float sample_freq, float cutoff_freq) ;
float Lpf2p_Apply(Lpf2p *lpf, float sample);
float middleFilter(float in_data);

float fast_atan2(float yy, float xx);

double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_sqrt(float number);
#endif
