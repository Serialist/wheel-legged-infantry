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
#include "Detect_Task.h"
#include "Remote_Control.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "b2b.h"

#define CHASSIS_CMD_ID 0x666

B2B_Chassis_Cmd_t ch_cmd;

void Detect_Task(void const *argument)
{
  uint8_t buf[8];

  for (;;)
  {
    FDCAN3_TxFrame.Header.Identifier = CHASSIS_CMD_ID;
    B2B_Chassis_Cmd_Encode(&ch_cmd, FDCAN3_TxFrame.Data);
    // USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

    Remote_Message_Moniter(&remote_ctrl);
    osDelay(1);
  }
}
