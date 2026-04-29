/**
 * @file Command_Task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-21
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "cmsis_os.h"

#include "Remote_Control.h"
#include "b2b.h"
#include "bsp_uart.h"
#include "rm_motor.h"
#include "vmc-dm.h"

#include "chassis.hpp"
#include "command-task.hpp"
#include "ins-task.hpp"
#include "observer.hpp"

B2B_Chassis_Command_t ch_cmd;

Ramp_t v_ramp; // 速度斜坡

extern Remote_Info_Typedef remote_ctrl;
extern Wheel_Leg_Target_t set;
extern Robo_Attitude_t att;
extern JUMP_State_t jump_state;

void Cmd_Get(void);
void Cmd_Reset(void);

// #define RC_SWITCH remote_ctrl.rc.s[DT7_SL]
#define RC_SWITCH ch_cmd.sw[0]

// #define RC_STOP DT7_UP
// #define RC_RUN DT7_MID
#define RC_STOP 0
#define RC_RUN 1

extern "C" void Command_Task(void const *argument)
{
	/* ================================ 系统初始化 ================================ */

	rbstate = RBS_INIT;

	/* ================================ 初始化 ================================ */

	Ramp_Init(&v_ramp, 0, -3.f, 3.f);

	rbstate = RBS_READY;

	/* ================================ 模式切换 ================================ */
	for (;;)
	{
		// 停止
		if (RC_SWITCH == RC_STOP)
		{
			rbstate = RBS_STOP;
			jump_state = JPS_NONE;
			Cmd_Reset();
		}
		// 正常行驶
		else if (RC_SWITCH == RC_RUN)
		//  || (RC_SWITCH == DT7_DOWN && jump_state != JPS_NONE))
		{
			rbstate = RBS_RUN;
			Cmd_Get();
		}
		// 跳
		// else if (RC_SWITCH == DT7_DOWN && jump_state == JPS_NONE)
		// {
		// 	rbstate = RBS_JUMP;
		// 	Cmd_Get();
		// }

		// USART_Vofa_Justfloat_Transmit(0, 0.f, 0.f);

		osDelay(1);
	}
}

// #define LEG_LEN_CMD remote_ctrl.rc.ch[DT7_RY]
// #define V_CMD remote_ctrl.rc.ch[DT7_LY]
// #define YAW_CMD remote_ctrl.rc.ch[DT7_LX]
// #define ROLL_CMD remote_ctrl.rc.ch[DT7_RX]

#define LEG_LEN_CMD ch_cmd.vy
#define V_CMD ch_cmd.vyaw
#define YAW_CMD ch_cmd.vx
#define ROLL_CMD 0

/// @brief 获取控制量
void Cmd_Get(void)
{
	set.v = Ramp_Update(&v_ramp, (V_CMD * 2.f / 660.0f), 0.003f);

	// set.x += V_CMD / 660 * 0.02f;

	set.yaw -= YAW_CMD * 0.0005f;

	set.length =
		LEG_LEN_CMD > 10 ? Remapf((float)LEG_LEN_CMD, 10.f, 660.f, 0.15f, 0.35f)
		: LEG_LEN_CMD < -10 ? Remapf(Clampf((float)LEG_LEN_CMD, -220.f, -10.f),
									 -220.f,
									 -10.f,
									 .1f,
									 .15f)
							: 0.15f; // deadzone

	// set.roll = -ROLL_CMD * 30.0f / 660.0f;
	set.roll = 0;

	if (set.v != 0)
		set.x = ob.x;
}

void Cmd_Reset(void)
{
	set.v = 0;
	set.yaw = ins.Yaw_TolAngle;
	set.roll = 0;
	set.x = ob.x;
}
