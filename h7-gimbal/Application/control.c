/**
 * @file Detect_Task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "cmsis_os.h"
#include "control.h"
#include "Remote_Control.h"
#include "bsp_can.h"
#include "INS_Task.h"

#include "b2b.h"
#include "shoot.h"
#include "gimbal.h"

#define CHASSIS_CMD_ID 0x666

Control_t control;

B2B_Chassis_Cmd_t ch_cmd;
BUFFER_T control_can_buf[8] = {0};

void Control_ZeroForce(void);
void Control_Running(void);

void Control_Task(void const *argument)
{
  uint8_t buf[8];

  for (;;)
  {
    // 检查模式 
    if (remote_ctrl.rc.s[DT7_SR] != DT7_DOWN ||
        remote_ctrl.rc_lost == true)
    {
      control.status = RBS_ZREOFORCE;
    }
    else
    {
      control.status = RBS_RUNNING;
    }

    // 控制逻辑
    switch (control.status)
    {
    default:
    case RBS_ZREOFORCE:
      shoot_control = Shoot_ZeroForce;
      gimbal_control = Gimbal_ZeroForce;
      Control_ZeroForce();
      break;

    case RBS_RUNNING:
      shoot_control = Shoot_Running;
      gimbal_control = Gimbal_Running;
      Control_Running();
      break;
    }

    B2B_Chassis_Cmd_Encode(&ch_cmd, control_can_buf);
    USER_FDCAN_Transmit(1, CHASSIS_CMD_ID, control_can_buf);

    Remote_Message_Moniter(&remote_ctrl);
    osDelay(1);
  }
}

void Control_ZeroForce(void)
{
  ch_cmd.vx = 0;
  ch_cmd.vyaw = INS_Info.Yaw_TolAngle;
  shoot.target.feed_freq = 0;
}

void Control_Running(void)
{
  // 底盘控制量
  ch_cmd.vx = remote_ctrl.rc.ch[DT7_LY] * 3.5f / 660.0f;
  ch_cmd.vyaw = remote_ctrl.rc.ch[DT7_RX] * 0.001f;
  shoot.target.feed_freq = (remote_ctrl.rc.ch[DT7_Z] > 10)    ? (remote_ctrl.rc.ch[DT7_Z] * 2 / 660)
                           : (remote_ctrl.rc.ch[DT7_Z] < -10) ? (remote_ctrl.rc.ch[DT7_Z] * 10 / 660)
                                                              : (0);

  gimbal.target.yaw += remote_ctrl.rc.ch[0] * 0.001f;
  gimbal.target.pitch += remote_ctrl.rc.ch[1] * 0.0005f;
}
