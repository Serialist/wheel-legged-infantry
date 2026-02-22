/************************
 * @file chassis.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief chassis class & methods
 * @version 0.1.0
 * @date 2025-12-01
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "chassis.h"

/**
 * attitude
 * status
 * flag
 * behavior
 * mode
 * command
 * control
 * target
 * current
 */

// config//
// typedef struct
// {
// 	uint32_t omni_motor[2]; // 电机反馈时间
// 	// uint32_t momentum_motor[2];
// 	uint32_t speed_set[2]; // 底盘速度设定时间
// 	uint32_t ree_power;	   // 裁判系统反馈功率时间
// 	uint32_t ree_statue;   // 裁判系统反馈机器人状态时间
// 	uint32_t model;
// 	uint32_t imu;
// 	uint32_t rc;
// } Time_t;

// enum Robo_Status
// {
//     ROBO_OFF,
//     ROBO_ON,

//     ROBO_STATUS_START,
//     ROBO_STATUS_STOP,

//     ROBO_STATUS_REBOOT,  // 进程重启
//     ROBO_STATUS_RESTART, // 软件整体重启
//     ROBO_STATUS_RESET,   // 硬件重启

//     ROBO_STATUS_SLEEP,

//     ROBO_STATUS_SHUTDOWN, // 关闭系统

//     ROBO_STATUS_INIT,    // 初始化中
//     ROBO_STATUS_WAITING, // 等待系统处理中，不能执行任何任务
//     ROBO_STATUS_READY,   // 就绪

//     ROBO_STATUS_IDLE,    // 空闲中
//     ROBO_STATUS_RUNNING, // 运行任务中
//     ROBO_STATUS_RESUME,  // 恢复中
//     ROBO_STATUS_PAUSE,   // 暂停中，可能有任务，可能空闲，但是都挂起

//     ROBO_STATUS_ERROR,     // 错误
//     ROBO_STATUS_WARNING,   // 警告
//     ROBO_STATUS_FATAL,     // 严重错误
//     ROBO_STATUS_CRITICAL,  // 紧急错误
//     ROBO_STATUS_EMERGENCY, // 紧急错误

// };

// enum Error_Level
// {
//     ERROR_BLOCKER,
//     ERROR_CRITICAL,
//     ERROR_MAJOR,
//     ERROR_MINOR,
// };

// enum Robo_Chassis_Mode
// {
//     ROBO_CHASSIS_MODE_NORMAL,
//     ROBO_CHASSIS_MODE_SLOW,
//     ROBO_CHASSIS_MODE_SPIN,
// };

// enum Robo_Control_Mode
// {
//     ROBO_MODE_CONTROL_NONE,    // 无控制，此时应该停止一切操作
//     ROBO_MODE_CONTROL_MANUAL,  // 手动，如实时遥控器
//     ROBO_MODE_CONTROL_AUTO,    // 自动，自主决策
//     ROBO_MODE_CONTROL_TASK,    // 预设任务流程
//     ROBO_MODE_CONTROL_COMMAND, // 预留给指令流
// };
