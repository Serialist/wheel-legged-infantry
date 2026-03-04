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
#include "b2b.h"
#include "shoot.h"

#define CHASSIS_CMD_ID 0x666

Control_t control;

B2B_Chassis_Cmd_t ch_cmd;
uint8_t control_can_buf[8] = {0};

void Control_Task(void const *argument)
{
  uint8_t buf[8];

  for (;;)
  {
    if (remote_ctrl.rc.s[DT7_SL] != DT7_DOWN ||
        remote_ctrl.rc_lost == true)
    {
      shoot_control = Shoot_ZeroForce;

      control.status = RBS_ZREOFORCE;
    }
    else
    {
      shoot_control = Shoot_Running;

      control.status = RBS_RUNNING;
    }

    // µ×ÅÌ¿ØÖÆÁ¿
    ch_cmd.vx = remote_ctrl.rc.ch[DT7_LY] * 3.5f / 660.0f;
    ch_cmd.vyaw = remote_ctrl.rc.ch[DT7_RX] * 0.001f;
    shoot.target.feed_freq = (remote_ctrl.rc.ch[DT7_Z] > 10)    ? (remote_ctrl.rc.ch[DT7_Z] * 2 / 660)
                             : (remote_ctrl.rc.ch[DT7_Z] < -10) ? remote_ctrl.rc.ch[DT7_Z] * 10 / 660
                                                                : (0);

    B2B_Chassis_Cmd_Encode(&ch_cmd, control_can_buf);

    USER_FDCAN_Transmit(1, CHASSIS_CMD_ID, control_can_buf);

    Remote_Message_Moniter(&remote_ctrl);
    osDelay(1);
  }
}
