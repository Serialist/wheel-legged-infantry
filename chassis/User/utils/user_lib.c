/**
 * @file userlib.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-06
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 快速求平方根倒数
/*
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1); // 此处缺 what the fuck（雾）
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/

/**
 * @brief          斜波函数初始化
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @retval         返回空
 */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
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
    return ramp_source_type->out;
}

/// @brief 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

/// @brief int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
 * @brief          最小二乘法初始化
 * @param[in]      最小二乘法结构体
 * @param[in]      样本数
 * @retval         返回空
 */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
 * @brief          最小二乘法拟合
 * @param[in]      最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
 * @brief          最小二乘法提取信号微分
 * @param[in]      最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 * @retval         返回斜率k
 */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
 * @brief          获取最小二乘法提取信号微分
 * @param[in]      最小二乘法结构体
 * @retval         返回斜率k
 */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
 * @brief          最小二乘法平滑信号
 * @param[in]      最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 * @retval         返回平滑输出
 */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
 * @brief          获取最小二乘法平滑信号
 * @param[in]      最小二乘法结构体
 * @retval         返回平滑输出
 */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

void slope_following(float *target, float *set, float acc)
{
    if (*target > *set)
    {
        *set = *set + acc;
        if (*set >= *target)
            *set = *target;
    }
    else if (*target < *set)
    {
        *set = *set - acc;
        if (*set <= *target)
            *set = *target;
    }
}

/**
 * @brief 快速幂算法 fast power
 *
 * @param a base
 * @param b exponent
 * @return long long
 *
 * @note a ^ b
 */
long long FPow(long long a, long long b)
{
    long long res = 1;

    while (b)
    {
        if (b & 1)
            res = res * a;
        a = a * a;
        b >>= 1;
    }
    return res;
}

/**
 * @brief 快速幂取模算法 fast power with mod
 *
 * @param a base
 * @param b exponent
 * @param p mod
 * @return long long
 *
 * @note a ^ b % p
 */
long long FPowMod(long long a, long long b, long long p)
{
    long long res = 1;

    while (b)
    {
        if (b & 1)
            res = res * a % p;
        a = a * a % p;
        b >>= 1;
    }
    return res;
}

/**
 * @brief 快速平方根
 *
 * @param x
 * @return float
 */
float FSqrtf(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int *)&x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

/**
 * @brief 计算最大公约数 greatest common divisor
 *
 * @param a
 * @param b
 * @return long long
 */
long long FGcd(long long a, long long b)
{
    if (b == 0)
        return a;
    return FGcd(b, a % b);
}

/// @brief 限幅
void Clamp(float *in, float min, float max)
{
    if (*in < min)
    {
        *in = min;
    }
    else if (*in > max)
    {
        *in = max;
    }
}

/// @brief 限幅
float Clampf(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

/// @brief 绝对值限幅
float ClampAbsf(float value, float max)
{
    return fminf(fmaxf(value, -max), max);
}

/**
 * @brief 循环限幅函数
 *
 * @param value
 * @param min
 * @param max
 * @return float
 *
 * @note 不做检查，min > max 未定义
 */
float LoopClampf(float value, float min, float max)
{
    // 实现 1
    // a mod b  ==  a-floor(a/b)*b
    float range = max - min;
    return min + value - floorf(value / range) * range;

    // 实现 2 （注意 -0 问题）
    // 注意 >= 而不是 >，因为符号位可能产生 -0.0 和 0.0
    // -0.0 > 0 是 false，而 0.0 > 0 是 true
    // value = fmodf(value - min, max - min);
    // return (value >= 0) ? (value + min) : (value + max);
}

/// @brief 值映射
float Remapf(float a, float inmin, float intmax, float outmin, float outmax)
{
    return outmin + (a - inmin) * (outmax - outmin) / (intmax - inmin);
}

/// @brief 斜坡函数
float Rampf(float x, float x0, float k_min, float k_max, float dt)
{
    return x + Clampf(x0 - x, k_min * dt, k_max * dt);
}

// 符号函数
float Signf(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}
