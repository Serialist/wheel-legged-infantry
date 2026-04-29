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

B2B_Chassis_Command_t ch_cmd;
uint8_t buf[8];

extern "C" void Command_Task(void const *argument)
{
	for (;;)
	{
		// ch_cmd.vx = VT13_Info.RC.Channel[0];
		// ch_cmd.vy = VT13_Info.RC.Channel[1];
		// ch_cmd.vyaw = VT13_Info.RC.Channel[2];
		// ch_cmd.sw[0] = VT13_Info.RC.Switch;
		// ch_cmd.button[0] = VT13_Info.RC.Left;
		// ch_cmd.button[1] = VT13_Info.RC.Right;
		// ch_cmd.button[2] = VT13_Info.RC.Stop;
		// ch_cmd.button[3] = VT13_Info.RC.Trigger;

		ch_cmd.vx = remote_ctrl.rc.ch[DT7_LX];
		ch_cmd.vy = remote_ctrl.rc.ch[DT7_RY];
		ch_cmd.vyaw = remote_ctrl.rc.ch[DT7_LY];
		ch_cmd.sw[0] = remote_ctrl.rc.s[DT7_SL] == DT7_UP ? 0 : 1;
		// ch_cmd.button[0] = VT13_Info.RC.Left;
		// ch_cmd.button[1] = VT13_Info.RC.Right;
		// ch_cmd.button[2] = VT13_Info.RC.Stop;
		// ch_cmd.button[3] = VT13_Info.RC.Trigger;

		B2B_Chassis_Command_Encode(&ch_cmd, buf);

		BSP_CAN_Transmit(BSP_PORT2, B2B_CHASSIS_CMD_ID, buf);

		osDelay(1);
	}
}
