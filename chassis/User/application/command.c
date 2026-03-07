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

extern B2B_Chassis_Cmd_t ch_cmd;
extern RC_ctrl_t rc_ctrl;
extern Wheel_Leg_Target_t set;
extern Robo_Attitude_t att;
extern JUMP_State_t jump_state;

typedef enum
{
	CMD_NONE = 0,
	CMD_CAN,
	CMD_DT7,
	CMD_FSI6,
	CMD_UPPER,
} Command_Type_t;

Command_Type_t cmd_type = CMD_DT7;

void Stop_Cmd(void);
void CAN_Get_Command(void);
void DT7_Get_Command(void);

void Command_Task(void const *argument)
{
	// void (*get_cmd)(void);

	// switch (cmd_type)
	// {
	// case NONE:
	// 	get_cmd = Stop_Cmd;

	// case CMD_CAN:
	// 	get_cmd = CAN_Get_Command;
	// 	break;
	// case CMD_DT7:
	// 	get_cmd = DT7_Get_Command;
	// 	break;

	// default:
	// 	break;
	// }

	for (;;)
	{
		// get_cmd();
		osDelay(1);
	}
}

void Stop_Cmd(void)
{
	// set.length = 0;
	set.x = ob.x;
	set.v = ob.v;
	set.yaw = att.yaw;
	set.roll = att.roll;
}

void CAN_Get_Command(void)
{
	set.v = ch_cmd.vx;
	set.yaw += ch_cmd.vyaw * 0.001f;
}

uint8_t prev_switch = 0;

void DT7_Get_Command(void)
{
	set.v = rc_ctrl.rc.ch[L_Y] * 3.5f / 660.0f;
	set.yaw -= rc_ctrl.rc.ch[L_X] * 0.001f;
	// set.length = rc_ctrl.rc.ch[R_Y] * 0.1f / 660.0f + 0.15f;
	set.roll = -rc_ctrl.rc.ch[R_X] * 45.0f / 660.0f;

	if (set.v != 0)
		set.x = ob.x;

	if (rc_ctrl.rc.s[S_L] == MID || prev_switch == DOWN) // 正常行驶
	{
		robo_status = RBS_RUN;
	}
	else if (rc_ctrl.rc.s[S_L] == DOWN && jump_state == JPS_NONE)
	{
		// robo_status = RBS_JUMP;
	}
	else
	{
		set.v = 0;
		set.yaw = att.totalyaw;
		set.roll = 0;
		set.x = ob.x;
	}

	prev_switch = rc_ctrl.rc.s[S_L];
}
