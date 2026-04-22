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
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "command-task.hpp"
#include "rm_motor.h"

extern RM_Motor_Feedback_t gm6020;

extern "C" void Command_Task(void const *argument)
{
	TickType_t Control_Task_SysTick = 0;

	for (;;)
	{
		Control_Task_SysTick = osKernelSysTick();

		RM_Motor_Control_Transmit(BSP_PORT2, GM6020_TX_V_ID_1, (RM_Motor_Control_t){0, 0, 0, 0});

		USART_Vofa_Justfloat_Transmit(0, 0.f, 0.f);

		osDelay(1);
	}
}
