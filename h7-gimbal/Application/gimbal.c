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
#include "control.h"
#include "rm_motor.h"
#include "fdcan.h"

Gimbal_t gimbal;

RM_Motor_Fdb_t pitch_motor_fdb = {0};

//                          KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
float yaw_pid_param[7] = {13.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
float pitch_pid_param[7] = {4000.f, 0, 40.f, 0, 0, 0, 16384.f};

PID_Info_TypeDef Yaw_PID;
PID_Info_TypeDef Pitch_PID;

float pitch_output = 0;
float yaw_output = 0;
float single_yaw_angle = 0;
float yaw_offset = 0;

RM_Motor_Cmd_t pitch_cmd;
RM_Motor_Cmd_t yaw_cmd;

uint8_t gb_can_buf[8] = {0};

TickType_t Gimbal_Task_SysTick = 0;

// FDCAN_Message_t gimbal_motor_msg;

void Gimbal_Task(void const *argument)
{

  PID_Init(&Yaw_PID, PID_POSITION, yaw_pid_param);
  PID_Init(&Pitch_PID, PID_POSITION, pitch_pid_param);

  osDelay(800);
  gimbal.target.pitch = INS_Info.Pitch_Angle;

  FDCAN_TxHeaderTypeDef gb_can_header = {
      .Identifier = 0x1FF,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = 8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0,
  };

  // CAN_Add_Device(&gimbal_motor_msg);

  for (;;)
  {
    Gimbal_Task_SysTick = osKernelSysTick();

    /* ================================ feedback ================================ */

    gimbal.feedback.pitch = INS_Info.Pitch_Angle;

    /* ================================ target ================================ */

    gimbal.target.yaw += remote_ctrl.rc.ch[0] * 0.01f;
    gimbal.target.pitch += remote_ctrl.rc.ch[1] * 0.0005f;

    gimbal.target.pitch = Clampf(gimbal.target.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);

    /* ================================ control ================================ */

    // yaw

    // single_yaw_angle = sLoopClampf(INS_Info.Yaw_TolAngle + yaw_offset, 0, 180);

    // yaw_output = yaw_pid_param[0] * (gimbal.target.yaw - single_yaw_angle) +
    //              yaw_pid_param[2] * (-INS_Info.Yaw_Gyro);
    // yaw_output = ClampAbsf(yaw_output, yaw_pid_param[6]);

    // pitch

    pitch_output = pitch_pid_param[0] * (gimbal.target.pitch - INS_Info.Pitch_Angle) +
                   pitch_pid_param[2] * (-INS_Info.Pitch_Gyro);
    gimbal.output.pitch = -(int16_t)ClampAbsf(pitch_output, pitch_pid_param[6]);

    if (control.status != RBS_RUNNING)
    {
      gimbal.output.pitch = 0;
      gimbal.output.yaw = 0;
      gimbal.target.yaw = INS_Info.Yaw_TolAngle;
      gimbal.target.pitch = INS_Info.Pitch_Angle;
    }

    /* ================================ output ================================ */

    pitch_cmd = gimbal.output.pitch;

    RM_Motor_Cmd_Encode(0, 0, pitch_cmd, 0, gb_can_buf);

    USER_FDCAN_Transmit(1, 0x1FF, gb_can_buf);

    osDelay(1);
  }
}

// USART_Vofa_Justfloat_Transmit(gimbal.feedback.yaw, gimbal.feedback.pitch, 0.f);