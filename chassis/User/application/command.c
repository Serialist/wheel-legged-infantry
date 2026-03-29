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

void Command_Task(void const *argument)
{
	while (!INS.ins_flag)
	{
		rbstate = RBS_INIT;
		osDelay(1);
	}

	rbstate = RBS_READY;

	for (;;)
	{
		Control_Get();

		osDelay(1);
	}
}

uint8_t last_switch = 0;

float aaa;

#define LEG_LEN_CMD rc_ctrl.rc.ch[R_Y]
#define V_CMD rc_ctrl.rc.ch[L_Y]
#define YAW_CMD rc_ctrl.rc.ch[L_X]
// #define ROLL_CMD rc_ctrl.rc.ch[R_X]

void Control_Get(void)
{

	set.v = Signf(V_CMD) * Ramp_Update(&v_ramp, fabsf(V_CMD * 3.f / 660.0f), 0.003f);
	set.yaw -= YAW_CMD * 0.0023f;

	// deadzone
	set.length = LEG_LEN_CMD > 10	 ? Remapf((float)LEG_LEN_CMD, 10.f, 660.f, 0.15f, 0.35f)
				 : LEG_LEN_CMD < -10 ? Remapf(Clampf((float)LEG_LEN_CMD, -220.f, -10.f), -220.f, -10.f, .1f, .15f)
									 : 0.15f;

	// set.roll = -rc_ctrl.rc.ch[R_X] * 30.0f / 660.0f;
	set.roll = 0;

	if (set.v != 0)
		set.x = ob.x;

	if (rc_ctrl.rc.s[S_L] == UP)
	{
		rbstate = RBS_STOP;

		set.v = 0;
		set.yaw = att.totalyaw;
		set.roll = 0;
		set.x = ob.x;
	}
	else if (rc_ctrl.rc.s[S_L] == MID || last_switch == DOWN) // 正常行驶
	{
		rbstate = RBS_RUN;
	}
	else if (rc_ctrl.rc.s[S_L] == DOWN && jump_state == JPS_NONE)
	{
		rbstate = RBS_JUMP;
	}

	last_switch = rc_ctrl.rc.s[S_L];
}

// Command_Encode(Command_t *self, uint8_t *buf)
// {
// 	buf[0] = (self->move << 4) |
// 			 (self->rot & 0x0F);
// 	buf[1] = (self->leg << 4) |
// 			 (self->shoot & 0x0F);
// }

// Command_Decode(Command_t *self, uint8_t *buf)
// {
// 	self->move = buf[0] >> 4;
// 	self->rot = buf[0] & 0x0F;
// 	self->leg = buf[1] >> 4;
// 	self->shoot = buf[1] & 0x0F;
// }
