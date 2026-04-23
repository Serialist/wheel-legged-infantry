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

#ifndef CHASSIS_H
#define CHASSIS_H

#include "utils.h"

// CAN ID
// 关节
#define HIP_LF_ID 0x03
#define HIP_LB_ID 0x04
#define HIP_RF_ID 0x01
#define HIP_RB_ID 0x02
// 轮毂
#define HUB_L_ID 0x201
#define HUB_R_ID 0x202
// 板间通信
#define B2B_CHASSIS_CMD_ID 0x666

// 机体姿态
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

// 机器人事件标志
typedef struct
{
	bool offground_l;
	bool offground_r;
	bool offground;
	bool fallen;
} Robo_Flag_t;

// 机器人状态
typedef enum
{
	RBS_NONE,
	RBS_INIT,  // 初始化
	RBS_READY, // 待命
	RBS_ERROR, // 错误
	RBS_RUN,   // 运行
	RBS_JUMP,  // 跳跃
	RBS_STOP   // 停止
} Robo_State_t;

// 跳跃状态机
typedef enum
{
	JPS_NONE,			 // 默认状态
	JPS_INIT,			 // 蹲下
	JPS_STRETCH,		 // 跳
	JPS_STRETCH_DAMPING, // 伸腿缓冲
	JPS_SHRINK,			 // 收腿
	JPS_SHRINK_DAMPING,	 // 收腿缓冲
	JPS_LAND			 // 缓冲落地
} JUMP_State_t;

// 目标值
typedef struct
{
	float x;			 // m   期望位置
	float v;			 // m/s 期望速度
	float yaw;			 // rad 期望 yaw
	float pitch;		 // rad 期望 pitch
	float roll;			 // rad 期望 roll
	float length;		 // m  期望腿长
	float height;		 // m  期望机体离地高度
	float jump_f0[2];	 // N 推力前馈，用于跳跃等
	float hip_torque[4]; // N*m 关节扭矩
	float hub_torque[2]; // N*m 轮毂扭矩
} Wheel_Leg_Target_t;

extern Robo_State_t rbstate;
extern Robo_Flag_t rbflag;

#endif
