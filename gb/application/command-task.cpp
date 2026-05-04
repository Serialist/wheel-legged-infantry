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
#include "Image_Transmission.h"

#include "gimbal.hpp"
#include "command-task.hpp"
#include "ins-task.hpp"
#include "bsp_can.h"

#include "superpower.h"
#include "math-utils.hpp"

extern RM_Motor_Feedback_t yaw_motor, pitch_motor;

B2B_Chassis_Command_t ch_cmd;
uint8_t buf[8];

void Command_ZeroForce(void);
void Command_Normal(void);
void Command_Jump(void);

float sp_target = 0;
uint8_t sp_buf[8];

extern "C" void Command_Task(void const *argument)
{

	for (;;)
	{
		Remote_Message_Moniter(&remote_ctrl);

		if (remote_ctrl.rc_lost == true || // 遥控器断线
			remote_ctrl.rc.s[DT7_SL] == DT7_UP)
		{
			Command_ZeroForce();
		}
		else if (remote_ctrl.rc.s[DT7_SL] == DT7_MID)
		{
			Command_Normal();
		}
		else if (remote_ctrl.rc.s[DT7_SL] == DT7_DOWN)
		{
			Command_Jump();
		}

		B2B_Chassis_Command_Encode(&ch_cmd, buf);

		// SuperPower_Cmd_Encode(sp_target, sp_buf);
		BSP_CAN_Transmit(BSP_PORT2, SUPERPOWER_CMD_ID, sp_buf);

		BSP_CAN_Transmit(BSP_PORT2, B2B_CHASSIS_CMD_ID, buf);

		osDelay(1);
	}
}

void Command_ZeroForce(void)
{
	gb_mode = Gimbal_Mode::zero_force;

	ch_cmd.vx = 0;
	ch_cmd.vy = 0;
	ch_cmd.vyaw = 0;
	ch_cmd.sw[0] = 0;

	yaw_position = ins.Yaw_TolAngle;
	pitch_position = ins.Pitch_Angle;
}

#define LEG_LEN_CMD remote_ctrl.rc.ch[DT7_WHEEL]
#define V_CMD remote_ctrl.rc.ch[DT7_LY]
#define VYAW_CMD remote_ctrl.rc.ch[DT7_RX]
#define VPITCH_CMD remote_ctrl.rc.ch[DT7_RY]

void Command_Normal(void)
{
	gb_mode = Gimbal_Mode::running;

	// ch_cmd.vx = VT13_Info.RC.Channel[0];
	// ch_cmd.vy = VT13_Info.RC.Channel[1];
	// ch_cmd.vyaw = VT13_Info.RC.Channel[2];
	// ch_cmd.sw[0] = VT13_Info.RC.Switch;
	// ch_cmd.button[0] = VT13_Info.RC.Left;
	// ch_cmd.button[1] = VT13_Info.RC.Right;
	// ch_cmd.button[2] = VT13_Info.RC.Stop;
	// ch_cmd.button[3] = VT13_Info.RC.Trigger;

	// B2B_Chassis_Command_Encode((B2B_Chassis_Command_t){.vx = remote_ctrl.rc.ch[DT7_LY],
	// 												   .vy = remote_ctrl.rc.ch[DT7_WHEEL],
	// 												   .vyaw = vgd::utils::Deadzonef(remote_ctrl.rc.ch[DT7_LX], 0, 60),
	// 												   .sw[0] = remote_ctrl.rc.s[DT7_SL] == DT7_UP ? 0 : 1},
	// 						   buf);

	yaw_position = yaw_position - VYAW_CMD * 0.0000075;

	pitch_position = vgd::utils::ClampAbsf(pitch_position + VPITCH_CMD * 0.000007, DEG2RAD(20));

	ch_cmd.vx = vgd::utils::Remapf(V_CMD, -660, 660, -2, 2);
	ch_cmd.vy = vgd::utils::Remapf(std::fmaxf(LEG_LEN_CMD, 60), 60, 660, 0.1, 0.4);
	ch_cmd.vyaw = yaw_position;
	ch_cmd.sw[0] = 1;
}

void Command_Jump(void)
{
	Command_Normal();
	ch_cmd.sw[0] = 2;
}
