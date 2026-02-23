/**
 ******************************************************************************
 * @file           : Control_Task.c
 * @brief          : Control task
 * @author         : GrassFan Wang
 * @date           : 2025/01/22
 * @version        : v1.1
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "INS_Task.h"

static void Control_Init(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Info_Update(Control_Info_Typedef *Control_Info);

Control_Info_Typedef Control_Info;

//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float yaw_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
static float pitch_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};

PID_Info_TypeDef Yaw_PID;
PID_Info_TypeDef Pitch_PID;

void Control_Task(void const *argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t Control_Task_SysTick = 0;

  Control_Init(&Control_Info);
  /* Infinite loop */
  for (;;)
  {
    Control_Task_SysTick = osKernelSysTick();

    Control_Measure_Update(&Control_Info);
    Control_Target_Update(&Control_Info);
    Control_Info_Update(&Control_Info);
    // USART_Vofa_Justfloat_Transmit(Control_Info.Measure.Chassis_Velocity, 0.f, 0.f);

    osDelay(1);
  }
}
/* USER CODE END Control_Task */

static void Control_Init(Control_Info_Typedef *Control_Info)
{
  PID_Init(&Yaw_PID, PID_POSITION, yaw_pid_param);
  PID_Init(&Pitch_PID, PID_POSITION, pitch_pid_param);
}

static void Control_Measure_Update(Control_Info_Typedef *Control_Info)
{
  Control_Info->Measure.yaw = INS_Info.Angle[IMU_ANGLE_INDEX_YAW];
  Control_Info->Measure.pitch = INS_Info.Angle[IMU_ANGLE_INDEX_PITCH];
}

static void Control_Target_Update(Control_Info_Typedef *Control_Info)
{
  Control_Info->Target.yaw += remote_ctrl.rc.ch[0] * 0.01f;
  Control_Info->Target.pitch += remote_ctrl.rc.ch[3] * 0.01f;

  VAL_LIMIT(Control_Info->Target.pitch,
            DegreesToRadians * PITCH_MIN_ANGLE,
            DegreesToRadians * PITCH_MAX_ANGLE);
}

static void Control_Info_Update(Control_Info_Typedef *Control_Info)
{
  PID_Calculate(&Yaw_PID, Control_Info->Target.yaw, Control_Info->Measure.yaw);
  PID_Calculate(&Pitch_PID, Control_Info->Target.pitch, Control_Info->Measure.pitch);

  Control_Info->SendValue[0] = (int16_t)(Yaw_PID.Output);
  Control_Info->SendValue[1] = (int16_t)(Pitch_PID.Output);
}
