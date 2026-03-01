/**
 * @file gimbal.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "gimbal.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "INS_Task.h"

Control_Info_Typedef Control_Info;

//                          KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
float yaw_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
float pitch_pid_param[7] = {4000.f, 0, 40.f, 0, 0, 0, 20000.f};

PID_Info_TypeDef Yaw_PID;
PID_Info_TypeDef Pitch_PID;

float pitch_output = 0;

TickType_t Control_Task_SysTick = 0;

void Control_Task(void const *argument)
{

  PID_Init(&Yaw_PID, PID_POSITION, yaw_pid_param);
  PID_Init(&Pitch_PID, PID_POSITION, pitch_pid_param);

  osDelay(100);
  Control_Info.target.pitch = INS_Info.Pitch_Angle;

  for (;;)
  {
    Control_Task_SysTick = osKernelSysTick();

    /* ================================ feedback ================================ */

    Control_Info.feedback.yaw = INS_Info.Yaw_TolAngle;
    Control_Info.feedback.pitch = INS_Info.Pitch_Angle;

    /* ================================ target ================================ */

    Control_Info.target.yaw += remote_ctrl.rc.ch[0] * 0.01f;
    Control_Info.target.pitch += remote_ctrl.rc.ch[1] * 0.0005f;

    Control_Info.target.pitch = Clampf(Control_Info.target.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);

    /* ================================ control ================================ */

    // Control_Info.output.yaw = (int16_t)PID_Calculate(&Yaw_PID, Control_Info.target.yaw, Control_Info.feedback.yaw);

    pitch_output = pitch_pid_param[0] * (Control_Info.target.pitch - Control_Info.feedback.pitch) +
                   pitch_pid_param[2] * (-INS_Info.Pitch_Gyro);
    Control_Info.output.pitch = -(int16_t)ClampAbsf(pitch_output, pitch_pid_param[6]);
    // Control_Info.output.pitch = 0;

    // FDCAN1_TxFrame.Header.Identifier = 0x1FF;
    // FDCAN1_TxFrame.Data[4] = (uint8_t)(Control_Info.output.pitch >> 8);
    // FDCAN1_TxFrame.Data[5] = (uint8_t)(Control_Info.output.pitch);
    // USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);

    // USART_Vofa_Justfloat_Transmit(Control_Info.feedback.yaw, Control_Info.feedback.pitch, 0.f);

    osDelay(1);
  }
}
