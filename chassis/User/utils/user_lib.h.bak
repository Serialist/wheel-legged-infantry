/**
 * @file userlib.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2026-01-12
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef USERLIB_H
#define USERLIB_H

/* ================================================================ include ================================================================ */

#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "arm_math.h"

// #include "cmsis_os.h" // 下面有这个要不要用（?）

/* ================================================================ macro ================================================================ */

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif /* _CMSIS_OS_H */
#endif /* user_malloc */

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define DEG_CLAMPF(Ang) LoopClampf((Ang), -180.0f, 180.0f) // 角度格式化为-180~180
#define RAD_CLAMPF(Ang) LoopClampf((Ang), -PI, PI)         // 弧度格式化为-PI~PI

#define DEG2RAD(Ang) ((Ang) * 0.01745329252f)
#define RAD2DEG(Ang) ((Ang) * 57.295779513f)

#define LF 0
#define LB 1
#define RF 2
#define RB 3

// 轮子
#define WL 4
#define WR 5

#define FRONT 0
#define BACK 1

#define LEFT 0
#define RIGHT 1

#define NONE 0

// 做一层 adapter，方便移植
/// @todo 放到单独一个适配层文件中，比如 math-adapter，这样不同平台移植方便还能硬件优化
#define SINF(x) arm_sin_f32(x)
#define COSF(x) arm_cos_f32(x)

/* ================================================================ typedef ================================================================ */

typedef enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
} GlobalDebugMode_t;

typedef struct
{
    float input;        // 输入数据
    float out;          // 输出数据
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间间隔
} ramp_function_source_t;

typedef __packed struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

/* ================================================================ variable ================================================================ */

/* ================================================================ prototype ================================================================ */

float Signf(float value);                                                     // 符号函数
void Clamp(float *in, float min, float max);                                  // 限幅
float Clampf(float value, float min, float max);                              // 限幅
float ClampAbsf(float value, float max);                                      // 绝对值限幅
float LoopClampf(float Input, float minValue, float maxValue);                // 循环限幅
float Remapf(float a, float inmin, float intmax, float outmin, float outmax); // 值映射
float Rampf(float prev_x, float x, float k_min, float k_max, float dt);       // 斜坡函数

float Sqrt(float x);                                      // 快速开方
long long FPow(long long a, long long b);                 // 快速幂
long long FPowMod(long long a, long long b, long long p); // 快速幂取模
float FSqrtf(float x);                                    // 快速平方根
long long FGcd(long long a, long long b);                 // 计算最大公约数 greatest common divisor

float float_deadband(float Value, float minValue, float maxValue);         // 浮点死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue); // int16 死区

float float_constrain(float Value, float minValue, float maxValue);         // 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue); // 限幅函数
int float_rounding(float raw);
void slope_following(float *target, float *set, float acc);

/* ================================ 斜波函数 ================================ */

void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min); // 斜波函数初始化
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);                             // 斜波函数计算

/* ================================ OLS 最小二乘法 ================================ */

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

/* ================================================================ function ================================================================ */

#endif
