/**
 * @file Control_Task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "shoot.h"
#include "cmsis_os.h"
#include "PID.h"
#include "Motor.h"

Shoot_Info_Typedef shoot_info;

//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float feed_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
static float frac1_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
static float frac2_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};

PID_Info_TypeDef feed_pid;
PID_Info_TypeDef frac1_pid;
PID_Info_TypeDef frac2_pid;

TickType_t shoot_task_tick = 0;
void Shoot_Task(void const *argument)
{

  PID_Init(&feed_pid, PID_POSITION, feed_pid_param);
  PID_Init(&frac1_pid, PID_POSITION, frac1_pid_param);
  PID_Init(&frac2_pid, PID_POSITION, frac2_pid_param);

  for (;;)
  {
    shoot_task_tick = osKernelSysTick();

    osDelay(1);
  }
}
