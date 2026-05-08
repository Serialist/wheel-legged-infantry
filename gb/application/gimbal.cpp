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

PID yaw_pid[2] = {{10, 0, 1, 15, 0}, {1500, 0, 0, 10000, 0}};
PID pitch_pid[2] = {{80, 0, 2, 7, 0}, {1500, 0.1, 0, 10000, 1000}};

float
	// yaw
	yaw_position, // 角度环 目标
	yaw_velocity, // 速度环 目标
	yaw_current,  // 电流输出
	// pitch
	pitch_position, // 角度环 目标
	pitch_velocity, // 速度环 目标
	pitch_current;	// 电流输出

Gimbal_Mode gb_mode = Gimbal_Mode::zero_force;

/* ================================================================ prototype ================================================================ */

void Gimbal_ZeroForce(void);
void Gimbal_Running(void);

/* ================================================================ function ================================================================ */

extern "C" void Gimbal_Task(void const *argument)
{
	for (;;)
	{
		switch (gb_mode)
		{
		case Gimbal_Mode::zero_force:
			Gimbal_ZeroForce();
			break;

		case Gimbal_Mode::running:
			Gimbal_Running();
			break;
		}

		osDelay(1);
	}
}

void Gimbal_ZeroForce(void)
{
	// RM_Motor_Control_Transmit(BSP_PORT2, GM6020_TX_V_ID_1, (RM_Motor_Control_t){0, 0, 0, 0}); // yaw tx
	// RM_Motor_Control_Transmit(BSP_PORT1, GM6020_TX_V_ID_1, (RM_Motor_Control_t){0, 0, 0, 0}); // pitch tx
}

void Gimbal_Running(void)
{
	// yaw 位置环
	yaw_velocity = yaw_pid[POSITION_PID].UpdateEZ(yaw_position - ins.Yaw_TolAngle, 0, -ins.Yaw_Gyro);

	// yaw 速度环
	yaw_current = yaw_pid[VELOCITY_PID].Update(yaw_velocity, ins.Yaw_Gyro);

	// yaw tx
	// yaw 电机反装向下，坐标系向上，所以方向反的
	RM_Motor_Control_Transmit(BSP_PORT2, GM6020_TX_V_ID_1, (RM_Motor_Control_t){0, 0, -(int16_t)yaw_current, 0});

	// pitch 位置环
	pitch_velocity = pitch_pid[POSITION_PID].UpdateEZ(pitch_position - ins.Pitch_Angle, 0, -ins.Pitch_Gyro);

	// pitch 速度环
	pitch_current = pitch_pid[VELOCITY_PID].Update(pitch_velocity, ins.Pitch_Gyro);

	// pitch tx
	RM_Motor_Control_Transmit(BSP_PORT1, GM6020_TX_V_ID_1, (RM_Motor_Control_t){0, 0, (int16_t)pitch_current, 0});
}
