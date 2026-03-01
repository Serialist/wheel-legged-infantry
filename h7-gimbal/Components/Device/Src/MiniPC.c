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

uint8_t usb_buf[256];
uint8_t usb_msg[256];
uint32_t usb_len = 0;

void MiniPC_Transmit_Info(uint8_t *Buff, const uint32_t *Len)
{
  CDC_Transmit_HS(Buff, *Len);
}

// usbd_cdc_if.c -> CDC_Receive_HS
void MiniPC_Recvive_Info(uint8_t *Buff, const uint32_t *Len)
{
  memcpy(usb_buf, Buff, *Len);
  usb_len = *Len;
  MiniPC_Transmit_Info(usb_buf, Len);
}