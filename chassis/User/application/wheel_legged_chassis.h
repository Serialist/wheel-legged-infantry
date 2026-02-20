/**
 * @file wheel_legged_chassis.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef Wheel_Legged_Chassis_H
#define Wheel_Legged_Chassis_H

#include "user_lib.h"

typedef struct
{
    // 机体加速度
    float ax;
    float az;
    // 机体角速度
    float yawspd;
    float pitchspd;
    float rollspd;
    // 欧拉角
    float yaw;
    float toatalyaw;
    float roll;
    float pitch;
} Robo_Attitude_t;

typedef struct Robo_Attitude_Def_t
{
    float yaw, v_yaw, a_yaw;
    float pitch, v_pitch, a_pitch;
    float roll, v_roll, a_roll;

    float x, v_x, a_x;
    float y, v_y, a_y;
    float z, v_z, a_z;

    float f_x, f_y, f_z;
    float m_x, m_y, m_z;
} Robo_Attitude_Def_t;

typedef struct Leg_Def_t
{
    float theta, v_theta;
    float alpha, v_lpha;

} Leg_Def_t;

typedef struct Chassis_Def_t
{
    Robo_Attitude_Def_t attitude;
    Leg_Def_t leg[2];

} Chassis_Def_t;

////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum Robo_Status_Def
{
    ROBO_STATUS_INIT,      // 初始化
    ROBO_STATUS_RUN,       // 运行
    ROBO_STATUS_STOP,      // 停止
    ROBO_STATUS_EMERGENCY, // 急停
    ROBO_STATUS_
} Robo_Status_Def;

typedef struct
{
    bool above_left;
    bool above_right;
    bool above;
    bool fallen;
} Wheel_Leg_Flag_t;

typedef struct
{
    Robo_Status_Def status;

    Wheel_Leg_Flag_t flag;
} Robo_Status_t;

typedef enum
{
    RBS_NONE,
    RBS_INIT,
    RBS_RUN,
    RBS_JUMP
} Robo_State_t;

typedef enum
{
    JS_NONE,
    JS_INIT,
    JS_STRETCH,
    JS_SHRINK,
    JS_AIR,
    JS_END
} JUMP_State_t;

typedef struct
{
    float x;                         // m   期望位置
    float v;                         // m/s 期望速度
    float yaw;                       // rad 期望 yaw
    float pitch;                     // rad 期望 pitch
    float roll;                      // rad 期望 roll
    float left_length, right_length; // m  期望腿长
    float height;
    float torque[6];
} Wheel_Leg_Target_t;

extern Robo_Status_t robo_status;

#endif
