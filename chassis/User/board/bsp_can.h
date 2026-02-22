/**
 * @file bsp_can.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "chassismotor.h"
#include "can.h"
#include "dji_motor.h"

extern DJI_RxData_Def_t m3508[2];

void can_filter_init(void);
float Uint_To_Float(int x_int, float x_min, float x_max, int bits);

void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);

#endif
