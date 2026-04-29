/**
 * @file application.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "task-tool.h"
#include "Minipc.h"

TASK_DEFINE(Gimbal_Task, osPriorityHigh, 2048);
TASK_DEFINE(Shoot_Task, osPriorityHigh, 2048);

void Application_Init(void)
{
	TASK_CREATE(Gimbal_Task);
	TASK_CREATE(Shoot_Task);
}
