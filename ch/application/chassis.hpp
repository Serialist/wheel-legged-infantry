/**
 * @file chassis.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "rm_motor.h"
#include "vmc-dm.h"

// 机器人事件标志
struct Robo_Flag_t {
    bool offground_l;
    bool offground_r;
    bool offground;
    bool fallen;
    bool reable;
    bool enable;
    bool spinbot;
    bool sp;
};

// 机体姿态
struct Robo_Attitude_t {
    float x, y, z;
    float vx, vy, vz;
    float ax, ay, az; // 机体加速度

    // 欧拉角
    float totalyaw;
    float roll, pitch, yaw;
    float vyaw, vpitch, vroll; // 机体角速度
};

// 机器人状态
enum Robo_State_t {
    RBS_NONE,
    RBS_INIT,  // 初始化
    RBS_READY, // 待命
    RBS_ERROR, // 错误
    RBS_RUN,   // 运行
    RBS_JUMP,  // 跳跃
    RBS_STOP   // 停止
};

// 跳跃状态机
enum JUMP_State_t {
    JPS_NONE,            // 默认状态
    JPS_INIT,            // 蹲下
    JPS_STRETCH,         // 跳
    JPS_STRETCH_DAMPING, // 伸腿缓冲
    JPS_SHRINK,          // 收腿
    JPS_SHRINK_DAMPING,  // 收腿缓冲
    JPS_LAND             // 缓冲落地
};

// 目标值
struct Wheel_Leg_Target_t {
    float x;             // m   期望位置
    float v;             // m/s 期望速度
    float yaw;           // rad 期望 yaw
    float vyaw;          // rad/s 期望 yaw
    float theta;         // rad
    float pitch;         // rad 期望 pitch
    float roll;          // rad 期望 roll
    float height;        // m  期望机体离地高度
    float length[2];     // m  期望腿长
    float jump_f0[2];    // N 推力前馈，用于跳跃等
    float hip_torque[4]; // N*m 关节扭矩
    float hub_torque[2]; // N*m 轮毂扭矩
};

extern Robo_State_t rbstate;
extern Robo_Flag_t rbflag;
extern RM_Motor_Feedback_t m3508[2];
extern VMC_t leg[2];
extern Wheel_Leg_Target_t set;

#endif
