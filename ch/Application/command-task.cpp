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

Ramp_t v_ramp; // 速度斜坡

B2B_Chassis_Cmd_t ch_cmd;

extern Remote_Info_Typedef remote_ctrl;
extern Wheel_Leg_Target_t set;
extern Robo_Attitude_t att;
extern JUMP_State_t jump_state;

void Cmd_Get(void);
void Cmd_Reset(void);

extern "C" void Command_Task(void const *argument)
{
	/* ================================ 系统初始化
	 * ================================ */

	rbstate = RBS_INIT;

	/* ================================ 初始化 ================================ */

	Ramp_Init(&v_ramp, 0, -3.f, 3.f);

	rbstate = RBS_READY;

	/* ================================ 模式切换 ================================
	 */
	for (;;)
	{
		// 停止
		if (remote_ctrl.rc.s[DT7_SL] == DT7_UP)
		{
			rbstate = RBS_STOP;
			jump_state = JPS_NONE;
			Cmd_Reset();
		}
		// 正常行驶
		else if (remote_ctrl.rc.s[DT7_SL] == DT7_MID
				 || (remote_ctrl.rc.s[DT7_SL] == DT7_DOWN
					 && jump_state != JPS_NONE))
		{
			rbstate = RBS_RUN;
			Cmd_Get();
		}
		// 跳
		else if (remote_ctrl.rc.s[DT7_SL] == DT7_DOWN && jump_state == JPS_NONE)
		{
			rbstate = RBS_JUMP;
			Cmd_Get();
		}

		// USART_Vofa_Justfloat_Transmit(0, 0.f, 0.f);

		osDelay(1);
	}
}

//
//
//
//

#define LEG_LEN_CMD remote_ctrl.rc.ch[DT7_RY]
#define V_CMD remote_ctrl.rc.ch[DT7_LY]
#define YAW_CMD remote_ctrl.rc.ch[DT7_LX]
#define ROLL_CMD remote_ctrl.rc.ch[DT7_RX]

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
	set.roll = 0.1;

	if (set.v != 0)
		set.x = ob.x;
}

void Cmd_Reset(void)
{
	set.v = 0;
	set.yaw = att.totalyaw;
	set.roll = 0;
	set.x = ob.x;
}
