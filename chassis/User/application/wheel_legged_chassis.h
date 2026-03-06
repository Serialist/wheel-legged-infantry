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

#define HIP_LF_ID 0x03
#define HIP_LB_ID 0x04
#define HIP_RF_ID 0x01
#define HIP_RB_ID 0x02
#define HUB_L_ID 0x207
#define HUB_R_ID 0x206
#define B2B_CHASSIS_CMD_ID 0x666

typedef struct
{
    float x, y, z;
    float vx, vy, vz;
    float ax, ay, az; // 机体加速度

    // 欧拉角
    float totalyaw;
    float roll, pitch, yaw;
    float vyaw, vpitch, vroll; // 机体角速度
} Robo_Attitude_t;

typedef struct
{
    bool above_left;
    bool above_right;
    bool above;
    bool fallen;
} Robo_Flag_t;

typedef enum
{
    RBS_NONE,
    RBS_INIT,
    RBS_RUN,
    RBS_JUMP
} Robo_Status_t;

typedef enum
{
    JPS_NONE,
    JPS_INIT,
    JPS_STRETCH,
    JPS_SHRINK,
    JPS_AIR,
    JPS_END
} JUMP_State_t;

typedef struct
{
    float x;      // m   期望位置
    float v;      // m/s 期望速度
    float yaw;    // rad 期望 yaw
    float pitch;  // rad 期望 pitch
    float roll;   // rad 期望 roll
    float length; // m  期望腿长
    float height;
    float torque[6];
} Wheel_Leg_Target_t;

extern Robo_Status_t robo_status;
extern Robo_Flag_t rbflag;

void Chassis_Task(void const *argument);

#endif
