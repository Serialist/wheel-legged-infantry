/**
 * @file Control_Task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "Command_Task.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "arm_math.h"

void Control_Task(void const *argument)
{
	TickType_t Control_Task_SysTick = 0;

	for (;;)
	{
		Control_Task_SysTick = osKernelSysTick();

		USART_Vofa_Justfloat_Transmit(0, 0.f, 0.f);

		osDelay(1);
	}
}
