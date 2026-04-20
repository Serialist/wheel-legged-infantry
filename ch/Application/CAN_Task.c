/**
 ******************************************************************************
 * @file           : CAN_Task.c
 * @brief          : CAN task
 * @author         : GrassFam Wang
 * @date           : 2025/1/22
 * @version        : v1.1
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "CAN_Task.h"

void CAN_Task(void const *argument)
{

	TickType_t CAN_Task_SysTick = 0;

	for (;;)
	{
		if (CAN_Task_SysTick % 2 == 0)
		{

			// 500Hz发送 请保证所有任务osDelay(1)
		}
		osDelay(1);
	}
}
