/**
 * @file b2b.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef B2B_H
#define B2B_H

#include "user_lib.h"

typedef struct
{
    float vx;
    float vy;
    float vyaw;
} B2B_Chassis_Cmd_t;

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

void B2B_Chassis_Cmd_Encode(B2B_Chassis_Cmd_t *data, uint8_t *buf);
void B2B_Chassis_Cmd_Decode(uint8_t *buf, B2B_Chassis_Cmd_t *data);

#endif
