/**
  * @file     math_calcu.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    数学计算函数
	*
  *	@author   Fatmouse
  *
  */
#include "math_calcu.h"
#include "prot_dr16.h"
#include "math.h"
#include "data_processing.h"
#include <stdlib.h>
#include <stdbool.h>
#define window 5 // 滑动窗口大小


#define M_PI_F 3.14159265358979f

ramp_function_source_t mwheel_ramp[4];
ramp_function_source_t chassis_w_ramp;
ramp_function_source_t chassis_super_x_ramp;
ramp_function_source_t chassis_super_y_ramp;
/**
  * @brief          斜波函数计算，根据输入的值进行叠加，输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->frame_period = frame_period;

    ramp_source_type->input = input;

    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void Slope_On(Slope_Struct *V)
{
    V->last_ticks = V->ticks;
    V->ticks = osKernelSysTick();
    if(V->real_target !=V->limit_target)
    {
        if(V->real_target < V->limit_target)//加操作
        {
            V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target > V->limit_target)//限幅
            {
                V->real_target =  V->limit_target;
            }
        }
        else if(V->real_target > V->limit_target)//减操作
        {
            V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target < V->limit_target)//限幅
            {
                V->real_target =  (short)V->limit_target;
            }
        }
    }
}

/**
  * @brief          冒泡排序函数
  * @param[in]      排序数组，数组大小
  * @retval         void
  */
//void Bubble_Sort(float *a,uint8_t n)
//{
//    float buf;
//    for(uint8_t i=0; i<n; ++i)
//    {
//        for(uint8_t j=0; j<n-1-i; ++j)
//        {
//            if(a[j]<a[j+1])
//            {
//                buf=a[j];
//                a[j] = a[j+1];
//                a[j+1] = buf;
//            }
//        }
//    }
//}

/**
  * @brief          滑动滤波函数   注：N2为滑动窗口大小
  * @author
  * @param[in]      滤波前的值、滤波缓存数组
  * @retval         滤波后的值
  */
float GildeAverageValueFilter(float NewValue,float *Data)
{
    float max,min;
    float sum;
    uint16_t i;
    Data[0]=NewValue;
    max=Data[0];
    min=Data[0];
    sum=Data[0];
    for(i=window-1; i!=0; i--)
    {
        if(Data[i]>max) max=Data[i];
        else if(Data[i]<min) min=Data[i];
        sum+=Data[i];
        Data[i]=Data[i-1];
    }
    i=window-2;
    sum=sum-max-min;
    sum=sum/i;
    return(sum);
}

/**
  * @brief          Sigmoid函数
  * @author
  * @param[in]      自变量
  * @retval         映射后的函数值
  */
float Sigmoid_function(float x)
{
    float y;
    float temp_x=ABS(x) - SIGMOID_MAX;			//将sigmoid函数向右平移最大区间值
    y= 1 / (1 + pow(e,-temp_x));
    return y;
}


/**
	*@func   float Circle_error(float set ,float get ,float circle_para)
	*@bref		环形数据计算偏差值
	*@param[in] set 设定值 get采样值 circle_para 一圈数值
	*@note	环形数据下，直接计算出PID中的偏差值
*/
float circle_error(float set,float get,float circle_para)
{
    float error;
    if(set > get)
    {
        if(set - get> circle_para/2)
            error = set - get - circle_para;
        else
            error = set - get;
    }
    else if(set < get)
    {
        if(set - get<-1*circle_para/2)
            error = set - get +circle_para;
        else
            error = set - get;
    }
    else	error = 0;

    return error;
}

float data_limit(float data, float max, float min)
{
    if(data >= max)					return max;
    else if(data <= min)		return min;
    else 										return data;
}

float lowpassfilter(float x)
{
	static float alpha = 0.95;
	static float y,y_prv;
	
	y = alpha * y_prv+ (1 - alpha) * x;
	y_prv = y;
	
	
	return y;

}

// Function to compare integers for qsort function
//int compare(const void *a, const void *b)
//{
//	return (*(int *)a - *(int *)b);
//}
// Function to perform median filter
//void medianFilter(float *signal, float *result, uint8_t N)
//{
//	int i, j, filterSize = 5;
//	int *temp = (int *)malloc(filterSize * sizeof(int));

//	for (i = 0; i < N; ++i)
//	{
//		for (j = 0; j < filterSize; ++j)
//		{
//			if ((i + j) < N)
//				temp[j] = signal[i + j];
//			else
//				temp[j] = 0;
//		}

//		qsort(temp, filterSize, sizeof(int), compare);
//		result[i] = temp[filterSize / 2];
//	}

//	free(temp);
//}






void Lpf2p_SetCutoffFreq(Lpf2p *lpf, float sample_freq, float cutoff_freq) {
    float fr = 0;
    float ohm = 0;
    float c = 0;

    fr = sample_freq / cutoff_freq;
    ohm = tanf(M_PI_F / fr);
    c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

    lpf->cutoff_freq1 = cutoff_freq;

    if (lpf->cutoff_freq1 > 0.0f) {
        lpf->b01 = ohm * ohm / c;
        lpf->b11 = 2.0f * lpf->b01;
        lpf->b21 = lpf->b01;
        lpf->a11 = 2.0f * (ohm * ohm - 1.0f) / c;
        lpf->a21 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Lpf2p_Apply(Lpf2p *lpf, float sample) {
    float delay_element_0 = 0, output = 0;
    if (lpf->cutoff_freq1 <= 0.0f) {
        // no filtering
        return sample;
    } else {
        delay_element_0 = sample - lpf->delay_element_11 * lpf->a11 - lpf->delay_element_21 * lpf->a21;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propagate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * lpf->b01 + lpf->delay_element_11 * lpf->b11 + lpf->delay_element_21 * lpf->b21;

        lpf->delay_element_21 = lpf->delay_element_11;
        lpf->delay_element_11 = delay_element_0;

        // return the value. Should be no need to check limits
        return output;
    }
}
/*int main() {
    Lpf2p lpf;
    Lpf2p_SetCutoffFreq(&lpf, 1000.0f, 100.0f); // Sample frequency of 1000 Hz, cutoff frequency of 100 Hz
    float filtered_value = Lpf2p_Apply(&lpf, 1.0f); // Applying filter to input value of 1.0
    return 0;
}*/
float data[5];

float middleFilter(float in_data)
{
    float sum = 0;
    float temp[5];
    float change;
    int i,j;
    //记录数据
    for(i=0; i<4; i++)
    {
        data[i]=data[i+1];
    }
    data[4] = in_data;
    //复制数据
    for(i=0; i<5; i++)
        temp[i] = data[i];
    //冒泡法排序
    for(i=1; i<5; i++)
        for(j=0; j<5-i; j++)
        {
            if(temp[i] > temp[i+1])
            {
                change = temp[i];
                temp[i] = temp[i+1];
                temp[i+1] = change;
            }
        }
    //求和
    for(i=1; i<4; i++)
        sum = sum + temp[i];
    //返回平均值
    return(sum/3);

}

float fast_atan_table[257] = 
{
	0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
	1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
	3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
	4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
	6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
	7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
	9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
	1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
	1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
	1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
	1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
	1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
	1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
	2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
	2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
	2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
	2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
	2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
	2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
	2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
	3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
	3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
	3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
	3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
	3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
	3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
	3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
	4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
	4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
	4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
	4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
	4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
	4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
	4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
	4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
	5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
	5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
	5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
	5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
	5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
	5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
	5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
	5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
	5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
	6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
	6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
	6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
	6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
	6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
	6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
	6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
	6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
	6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
	6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
	7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
	7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
	7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
	7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
	7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
	7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
	7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
	7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
	7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
	7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
	7.853983e-01
};

float fast_atan2(float yy, float xx) 
{
	float x_abs, y_abs, zz;
	float alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if(xx == 0.0f)
		angle = MY_PPPIII_HALF;
	else if(yy == 0.0f)
		return 0;
	{
		/* normalize to +/- 45 degree range */
		y_abs = ABS(yy);
		x_abs = ABS(xx);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			zz = y_abs / x_abs;
		else
			zz = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (zz < TAN_MAP_RES)
			base_angle = zz;
		else 
		{
			/* find index and interpolation value */
			alpha = zz * (float) TAN_MAP_SIZE - .5f;
			index = (int) alpha;
			alpha -= (float) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (xx >= 0.0f) 
			{           /* -45 -> 45 */
				if (yy >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (yy >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (yy >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (xx >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (xx >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}


#ifdef ZERO_TO_TWOPI
	if (angle < 0)
		return (angle + TWOPI);
	else
		return (angle);
#else
	return (angle);
#endif
}


//快速平方根算法
//函数名：invSqrt(void)
//描述：求平方根的倒数
//该函数是经典的Carmack求平方根算法，效率极高，使用魔数0x5f375a86
float my_sqrt(float number)
{
	long i;
	float xx, yy;
	const float f = 1.5F;
	xx = number * 0.5F;
	yy = number;
	i = * ( long * ) &yy;
	i = 0x5f375a86 - ( i >> 1 );

	yy = * ( float * ) &i;
	yy = yy * ( f - ( xx * yy * yy ) );
	yy = yy * ( f - ( xx * yy * yy ) );
	return number * yy;
}

#define ONE_PI   (3.14159265)
#define TWO_PI   (2.0 * 3.14159265)
#define ANGLE_UNIT (TWO_PI/10.0)

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	int8_t flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

float my_cos(double rad)
{
	int8_t flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin(rad)*flag;
}


//float data[10];

//float middleFilter(float in_data)
//{
//    float sum = 0;
//    float temp[10];
//    float change;
//    int i,j;
//    //记录数据
//    for(i=0; i<9; i++)
//    {
//        data[i]=data[i+1];
//    }
//    data[9] = in_data;
//    //复制数据
//    for(i=0; i<10; i++)
//        temp[i] = data[i];
//    //冒泡法排序
//    for(i=1; i<10; i++)
//        for(j=0; j<10-i; j++)
//        {
//            if(temp[i] > temp[i+1])
//            {
//                change = temp[i];
//                temp[i] = temp[i+1];
//                temp[i+1] = change;
//            }
//        }
//    //求和
//    for(i=1; i<9; i++)
//        sum = sum + temp[i];
//    //返回平均值
//    return(sum/8);

//}


//float data[15];

//float middleFilter(float in_data)
//{
//    float sum = 0;
//    float temp[15];
//    float change;
//    int i,j;
//    //记录数据
//    for(i=0; i<14; i++)
//    {
//        data[i]=data[i+1];
//    }
//    data[14] = in_data;
//    //复制数据
//    for(i=0; i<15; i++)
//        temp[i] = data[i];
//    //冒泡法排序
//    for(i=1; i<15; i++)
//        for(j=0; j<15-i; j++)
//        {
//            if(temp[i] > temp[i+1])
//            {
//                change = temp[i];
//                temp[i] = temp[i+1];
//                temp[i+1] = change;
//            }
//        }
//    //求和
//    for(i=1; i<15; i++)
//        sum = sum + temp[i];
//    //返回平均值
//    return(sum/13);

//}
