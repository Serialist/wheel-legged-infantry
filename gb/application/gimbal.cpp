/**
 * @file gimbal.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-26
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 * oooooo     oooo   .oooooo.    oooooooooo.
 *  `888.     .8'   d8P'  `Y8b   `888'   `Y8b
 *   `888.   .8'   888            888      888
 *    `888. .8'    888            888      888
 *     `888.8'     888     ooooo  888      888
 *      `888'      `88.    .88'   888     d88'
 *       `8'        `Y8bood8P'   o888bood8P'
 * 
 */

/* ================================================================ include ================================================================ */

#include "cmsis_os.h"
#include "ptpid.hpp"
#include "ins-task.hpp"
#include "rm_motor.h"
#include "gimbal.hpp"

using namespace vgd;

/* ================================================================ micro ================================================================ */

#define POSITION_PID 0
#define VELOCITY_PID 1

/* ================================================================ variable ================================================================ */

RM_Motor_Feedback_t yaw_motor, pitch_motor;

PT::PID yaw_pid[2] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
PT::PID pitch_pid[2] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};

float
	// yaw
	yaw_position, // 角度环 目标
	yaw_velocity, // 速度环 目标
	yaw_current,  // 电流输出
	// pitch
	pitch_position, // 角度环 目标
	pitch_velocity, // 速度环 目标
	pitch_current;	// 电流输出

/* ================================================================ prototype ================================================================ */

/* ================================================================ function ================================================================ */

extern "C" void Gimbal_Task(void const *argument)
{
	for (;;)
	{
		yaw_velocity = yaw_pid[POSITION_PID].UpdateEZ(
			yaw_position - ins.Yaw_Angle, 0, -ins.Yaw_Gyro);

		yaw_current = yaw_pid[VELOCITY_PID].Update(yaw_velocity, ins.Yaw_Gyro);

		osDelay(1);
	}
}

// RM_Motor_Fdb_t pitch_motor_fdb = {0};

// //                          KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
// float yaw_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
// float pitch_pid_param[7] = {4000.f, 0, 40.f, 0, 0, 0, 16384.f};

// float pitch_output = 0;
// float yaw_output = 0;
// float single_yaw_angle = 0;

// RM_Motor_Cmd_t pitch_cmd, yaw_cmd;

// BUFFER_T gb_can_buf[8];

// TickType_t Gimbal_Task_SysTick = 0;

// void (*gimbal_control)(void) = Gimbal_ZeroForce;

// void Gimbal_Task(void const *argument)
// {

// 	PID_Init(&Yaw_PID, PID_POSITION, yaw_pid_param);
// 	PID_Init(&Pitch_PID, PID_POSITION, pitch_pid_param);

// 	gimbal.target.pitch = INS_Info.Pitch_Angle;

// 	for (;;)
// 	{
// 		Gimbal_Task_SysTick = osKernelSysTick();

// 		gimbal_control();

// 		RM_Motor_Cmd_Encode(0, 0, pitch_cmd, 0, gb_can_buf);
// 		USER_FDCAN_Transmit(1, 0x1FF, gb_can_buf);

// 		osDelay(1);
// 	}
// }

// void Gimbal_ZeroForce(void)
// {
// 	gimbal.output.pitch = 0;
// 	gimbal.output.yaw = 0;
// 	gimbal.target.yaw = INS_Info.Yaw_TolAngle;
// 	gimbal.target.pitch = INS_Info.Pitch_Angle;
// }

// float yaw_offset = 0;

// void Gimbal_Running(void)
// {
// 	// yaw

// 	single_yaw_angle = LoopClampf(INS_Info.Yaw_TolAngle + yaw_offset, 0, 180);

// 	yaw_output = yaw_pid_param[0] * (gimbal.target.yaw - single_yaw_angle)
// 				 + yaw_pid_param[2] * (-INS_Info.Yaw_Gyro);
// 	yaw_output = ClampAbsf(yaw_output, yaw_pid_param[6]);

// 	// pitch

// 	gimbal.target.pitch =
// 		Clampf(gimbal.target.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);

// 	pitch_output =
// 		pitch_pid_param[0] * (gimbal.target.pitch - INS_Info.Pitch_Angle)
// 		+ pitch_pid_param[2] * (-INS_Info.Pitch_Gyro);
// 	gimbal.output.pitch = -(int16_t)ClampAbsf(pitch_output, pitch_pid_param[6]);

// 	pitch_cmd = gimbal.output.pitch;
// }
