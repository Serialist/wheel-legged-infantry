/**
 * @file dt7.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 遥控器处理
 * @version 0.1.0
 * @date 2026-02-10
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 * @note
 * 遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
 * 资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
 * 的方式保证热插拔的稳定性。
 * 
*/

/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       dt7.c/h
  * @brief      
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */


#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"
#include "bsp_rc.h"

#define RC_IS_OFFLINE(rc_ctrl) ((rc_ctrl)->offline_flag == 1)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
#define R_X 0
#define R_Y 1
#define L_X 2
#define L_Y 3
#define L_Z 4

#define S_L 1
#define S_R 0

#define UP 1
#define MID 3
#define DOWN 2

#define R_X 0
#define R_Y 1
#define L_X 2
#define L_Y 3
#define L_Z 4

typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

        uint8_t receive_flag;
        uint8_t offline_flag;
        uint16_t time_count;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern RC_ctrl_t rc_ctrl;

void remote_control_init(void);
void RC_Offline_Detection(RC_ctrl_t *rc_ctrl, uint32_t dt);

#endif
