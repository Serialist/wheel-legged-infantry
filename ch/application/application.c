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

#include "Minipc.h"
#include "stddef.h"
#include "task-tool.h"

TASK_DEFINE(Chassis_Task, osPriorityHigh, 2048);

StackType_t powerControllTaskStack[128];
StaticTask_t powerControllTaskTCB;
void powerControllTask(void* pvParameters);
void Application_Init(void) {
    TASK_CREATE(Chassis_Task);

    xTaskCreateStatic(
        powerControllTask,
        "PowerControll",
        128,
        NULL,
        10,
        powerControllTaskStack,
        &powerControllTaskTCB
    );
}
