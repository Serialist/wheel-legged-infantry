/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : MiniPC.c
 * @brief          : MiniPC interfaces functions
 * @author         : GarssFan Wang
 * @date           : 2025/01/22
 * @version        : v1.0
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"

uint8_t terminal_buf[256] = {0};
uint32_t terminal_len = 0;

void MiniPC_Transmit_Info(uint8_t *Buff, const uint32_t Len)
{
  CDC_Transmit_HS(Buff, Len);
}

void MiniPC_Recvive_Info(uint8_t *Buff, const uint32_t Len)
{
  if (Buff[0] == '\r')
  {
    memcpy(terminal_buf, "\n\rOK\n\r\n", 7);
    CDC_Transmit_HS(terminal_buf, 7);
  }
  else
  {
    terminal_buf[0] = Buff[0];
    CDC_Transmit_HS(terminal_buf, 1);
  }
}
