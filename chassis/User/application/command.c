/**
 * @file command.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "command.h"
#include "main.h"
#include "cmsis_os.h"
#include "b2b.h"
#include "dt7.h"
#include "wheel_legged_chassis.h"
#include "observer.h"

Ramp_t v_ramp; // 速度斜坡

uint8_t last_switch = 0;

extern B2B_Chassis_Cmd_t ch_cmd;
extern RC_ctrl_t rc_ctrl;
extern Wheel_Leg_Target_t set;
extern Robo_Attitude_t att;
extern JUMP_State_t jump_state;

void Cmd_Get(void);
void Cmd_Reset(void);

void Command_Task(void const *argument)
{
	/* ================================ 系统初始化 ================================ */

	rbstate = RBS_INIT;

	for (int i = 0;; i++)
	{
		if (INS.ready == true)
		{
			rbstate = RBS_READY;
			break;
		}
		else if (i > 1000)
		{
			rbstate = RBS_ERROR;
			break;
		}
		osDelay(1);
	}

	/* ================================ 自己初始化 ================================ */

	Ramp_Init(&v_ramp, 0, -15.f, 5.f);

	rbstate = RBS_READY;

	/* ================================ 模式切换 ================================ */
	for (;;)
	{
		// 停止
		if (rc_ctrl.rc.s[S_L] == UP)
		{
			rbstate = RBS_STOP;
			Cmd_Reset();
		}
		// 正常行驶
		else if (rc_ctrl.rc.s[S_L] == MID ||
				 (rc_ctrl.rc.s[S_L] == DOWN && jump_state != JPS_NONE))
		{
			rbstate = RBS_RUN;
			Cmd_Get();
		}
		// 跳
		else if (rc_ctrl.rc.s[S_L] == DOWN && jump_state == JPS_NONE)
		{
			rbstate = RBS_JUMP;
			Cmd_Get();
		}

		last_switch = rc_ctrl.rc.s[S_L];

		osDelay(1);
	}
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

#define LEG_LEN_CMD rc_ctrl.rc.ch[R_Y]
#define V_CMD rc_ctrl.rc.ch[L_Y]
#define YAW_CMD rc_ctrl.rc.ch[L_X]
#define ROLL_CMD rc_ctrl.rc.ch[R_X]

/// @brief 获取控制量
void Cmd_Get(void)
{
	set.v = Signf(V_CMD) * Ramp_Update(&v_ramp, fabsf(V_CMD * 3.f / 660.0f), 0.003f);

	set.yaw -= YAW_CMD * 0.0005f;

	set.length = LEG_LEN_CMD > 10	 ? Remapf((float)LEG_LEN_CMD, 10.f, 660.f, 0.15f, 0.35f)
				 : LEG_LEN_CMD < -10 ? Remapf(Clampf((float)LEG_LEN_CMD, -220.f, -10.f), -220.f, -10.f, .1f, .15f)
									 : 0.15f; // deadzone

	// set.roll = -ROLL_CMD * 30.0f / 660.0f;
	set.roll = 0;

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
