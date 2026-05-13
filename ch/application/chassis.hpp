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
	JPS_NONE,			 // 默认状态
	JPS_INIT,			 // 蹲下
	JPS_STRETCH,		 // 跳
	JPS_STRETCH_DAMPING, // 伸腿缓冲
	JPS_SHRINK,			 // 收腿
	JPS_SHRINK_DAMPING,	 // 收腿缓冲
	JPS_LAND			 // 缓冲落地
};

// 目标值
struct Wheel_Leg_Target_t {
	float x;			 // m   期望位置
	float v;			 // m/s 期望速度
	float yaw;			 // rad 期望 yaw
	float vyaw;			 // rad/s 期望 yaw
	float theta;		 // rad
	float pitch;		 // rad 期望 pitch
	float roll;			 // rad 期望 roll
	float height;		 // m  期望机体离地高度
	float length[2];	 // m  期望腿长
	float jump_f0[2];	 // N 推力前馈，用于跳跃等
	float hip_torque[4]; // N*m 关节扭矩
	float hub_torque[2]; // N*m 轮毂扭矩
};

extern Robo_State_t rbstate;
extern Robo_Flag_t rbflag;
extern RM_Motor_Feedback_t m3508[2];
extern VMC_t leg[2];
extern Wheel_Leg_Target_t set;

// typedef enum
// {
//     CMT_NONE = 0,
//     CMT_B2B,   // 板间通信
//     CMT_DT7,   // DT7 遥控器
//     CMT_FSI6,  // 富斯 i6 遥控器
//     CMT_UPPER, // 上位机指令
// } Command_Type_t;

// typedef enum
// {
//     MOVE_CMD_NONE = 0,
//     MOVE_CMD_STOP,   // 原地静止
//     MOVE_CMD_SLOW,   // 慢速
//     MOVE_CMD_NORMAL, // 正常
//     MOVE_CMD_FAST,   // 急速
// } Move_Cmd_t;

// typedef enum
// {
//     ROT_CMD_NONE = 0,
//     ROT_CMD_STOP,         // 直线走
//     ROT_CMD_LEFT,         // 左转
//     ROT_CMD_RIGHT,        // 右转
//     ROT_CMD_SPINBOT,      // 小陀螺
//     ROT_CMD_SPINBOT_FAST, // 快速小陀螺
// } Rotation_Cmd_t;

// typedef enum
// {
//     LEG_CMD_NONE = 0,
//     LEG_CMD_STOP,   // 推力 无力
//     LEG_CMD_SHORT,  // 短模式
//     LEG_CMD_NORMAL, // 正常
//     LEG_CMD_LONG,   // 长腿模式
//     LEG_CMD_JUMP,   // 跳跃
// } Leg_Cmd_t;

// typedef enum
// {
//     SHOOT_CMD_NONE = 0,
//     SHOOT_CMD_STOP,          // 停止
//     SHOOT_CMD_READY,         // 开摩擦轮（就绪）
//     SHOOT_CMD_RELOAD,        // 退弹
//     SHOOT_CMD_FIRE_SINGLE,   // 单发
//     SHOOT_CMD_FIRE_FULLAUTO, // 连发
//     SHOOT_CMD_AIMBOT         // 开自瞄
// } Shoot_Cmd_t;

// typedef enum
// {
//     MDS_NONE = 0,
//     MDS_INIT,    // 初始化
//     MDS_READY,   // 就绪
//     MDS_RUN,     // 运行
//     MDS_STOP,    // 停止
//     MDS_STANDBY, // 待命
//     MDS_ERROR,   // 错误
// } Module_State_t;

// typedef enum
// {
//     ERR_NONE = 0,
//     ERR_WARNING, // 警告，不影响正常功能
//     ERR_ERROR,   // 错误，影响正常功能，但还可以运行
//     ERR_FAULT    // 故障，无法正常运行，立刻重启
// } Error_Code_t;

// typedef struct
// {
//     Move_Cmd_t move : 4;
//     Rotation_Cmd_t rot : 4;
//     Leg_Cmd_t leg : 4;
//     Shoot_Cmd_t shoot : 4;
// } Command_t;

#endif
