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
	for (;;)
	{
		osDelay(1);
	}
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
