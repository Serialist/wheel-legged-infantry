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
#include "CAN_Task.h"
#include "gimbal.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"

extern int16_t ttttt;

void CAN_Task(void const *argument)
{
  TickType_t CAN_Task_SysTick = 0;

  for (;;)
  {

    CAN_Task_SysTick = osKernelSysTick();

    memset(&FDCAN1_TxFrame.Data, 0, 8);
    FDCAN1_TxFrame.Header.Identifier = 0x1FF;
    FDCAN1_TxFrame.Data[4] = (uint8_t)(Control_Info.output.pitch >> 8);
    FDCAN1_TxFrame.Data[5] = (uint8_t)(Control_Info.output.pitch);
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);

    memset(&FDCAN1_TxFrame.Data, 0, 8);
    FDCAN1_TxFrame.Header.Identifier = 0x200;
    FDCAN1_TxFrame.Data[0] = (uint8_t)(ttttt >> 8);
    FDCAN1_TxFrame.Data[1] = (uint8_t)(ttttt);
    FDCAN1_TxFrame.Data[2] = (uint8_t)(ttttt >> 8);
    FDCAN1_TxFrame.Data[3] = (uint8_t)(ttttt);
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);

    if (CAN_Task_SysTick % 2 == 0)
    {

      // 500Hz发送 请保证所有任务osDelay(1)
    }

    osDelay(1);
  }
}
