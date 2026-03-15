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
#endif
#endif

#ifndef MAT
#define MAT arm_matrix_instance_f32
#define MAT_INIT arm_mat_init_f32
#define MAT_ADD arm_mat_add_f32
#define MAT_SUB arm_mat_sub_f32
#define MAT_MULT arm_mat_mult_f32
#define MAT_TRANS arm_mat_trans_f32
#define MAT_INVERSE arm_mat_inverse_f32
#endif

#ifndef PI
#define PI 3.14159265354f
#endif

#ifndef MIN
#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#endif

#define DEG_CLAMPF(Ang) LoopClampf((Ang), -180.0f, 180.0f) // 角度格式化为-180~180
#define RAD_CLAMPF(Ang) LoopClampf((Ang), -PI, PI)         // 弧度格式化为-PI~PI

#ifndef DEG2RAD
#define DEG2RAD(Ang) ((Ang) * 0.01745329252f)
#endif

#ifndef RAD2DEG
#define RAD2DEG(Ang) ((Ang) * 57.295779513f)
#endif

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

#ifndef NONE
#define NONE 0
#endif

#ifndef NULL
#define NULL 0
#endif
// #define AIMASSIST 1

// 做一层 adapter，方便移植
/// @todo 放到单独一个适配层文件中，比如 math-adapter，这样不同平台移植方便还能硬件优化
#define SINF(x) arm_sin_f32(x)
#define COSF(x) arm_cos_f32(x)

#define BUFFER_T __attribute__((section(".AXI_SRAM"))) static uint8_t

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

/* ================================================================ variable ================================================================ */

/* ================================================================ prototype ================================================================ */

float Signf(float value);                                                    // 符号函数
void Clampfp(float *in, float min, float max);                               // 指针限幅
float Clampf(float value, float min, float max);                             // 限幅
float ClampAbsf(float value, float max);                                     // 绝对值限幅
float LoopClampf(float Input, float minValue, float maxValue);               // 循环限幅
float Remapf(float a, float inmin, float inmax, float outmin, float outmax); // 值映射
float Rampf(float prev_x, float x, float k_min, float k_max, float dt);      // 斜坡函数
float Deadzonef(float value, float point, float deadzone);                   // 死区

float Modf(float value, float range);

float SSqrt(float x);                                     // 开方
long long FPow(long long a, long long b);                 // 快速幂
long long FPowMod(long long a, long long b, long long p); // 快速幂取模
float FiSqrt(float x);                                    // 快速平方根倒数
float FSqrtf(float x);                                    // 快速平方根
long long FGcd(long long a, long long b);                 // 计算最大公约数 greatest common divisor

/* ================================ 斜波函数 ================================ */

typedef struct
{
    float value; // 输出数据
    float kmin;  // 斜率最小值
    float kmax;  // 斜率最大值
} Ramp_t;

void Ramp_Init(Ramp_t *self, float initial_value, float kmin, float kmax); // 斜波函数初始化
void Ramp_Reset(Ramp_t *self, float value);                                // 重置
float Ramp_Update(Ramp_t *self, float target, float dt);                   // 斜波函数计算

/* ================================ OLS Ordinary Least Squares 最小二乘法 ================================ */

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
} OLS_t;

void OLS_Init(OLS_t *OLS, uint16_t order);
void OLS_Update(OLS_t *OLS, float deltax, float y);
float OLS_Derivative(OLS_t *OLS, float deltax, float y);
float OLS_Smooth(OLS_t *OLS, float deltax, float y);
float Get_OLS_Derivative(OLS_t *OLS);
float Get_OLS_Smooth(OLS_t *OLS);

/* ================================================================ function ================================================================ */

#endif
