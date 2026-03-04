/**
 * @file rm_motor.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-01
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

/* ================================================================ include ================================================================ */

#include "rm_motor.h"

/* ================================================================ macro ================================================================ */

/* ================================================================ struct ================================================================ */

/* ================================================================ variable ================================================================ */

// RM_Motor_t *rm_motor_registry = NULL;

// bool register_error_flag = false;

/* ================================================================ prototype ================================================================ */

/* ================================================================ function ================================================================ */

void RM_Motor_Cmd_Encode(int16_t data1, int16_t data2, int16_t data3, int16_t data4, uint8_t *buf)
{
    buf[0] = data1 >> 8;
    buf[1] = data1;
    buf[2] = data2 >> 8;
    buf[3] = data2;
    buf[4] = data3 >> 8;
    buf[5] = data3;
    buf[6] = data4 >> 8;
    buf[7] = data4;
}

void RM_Motor_Fdb_Decode(uint8_t *buf, RM_Motor_Fdb_t *data)
{
    data->angle = (int16_t)((buf[0] << 8) | buf[1]);
    data->speed = (int16_t)((buf[2] << 8) | buf[3]);
    data->current = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp = buf[6];
}

// bool RM_Motor_Register(RM_Motor_t *motor, RM_Motor_Type_t type, uint8_t id)
// {
//     if (id > 8 ||
//         type >= 4)
//     {
//         register_error_flag = true;
//         return false;
//     }

//     int16_t my_can_id = 0;

//     switch (type)
//     {
//     case RM_C620:
//         my_can_id = C620_RX_ID(id);
//         break;

//     case RM_C610:
//         my_can_id = C610_RX_ID(id);
//         break;

//     case RM_GM6020:
//         my_can_id = GM6020_RX_ID(id);
//         break;

//     default:
//         register_error_flag = true;
//         return false;
//     }

//     RM_Motor_t *p = rm_motor_registry;
//     uint32_t canid = 0;
//     for (;;)
//     {
//         switch (p->type)
//         {
//         case RM_C620:
//             canid = C620_RX_ID(id);
//             break;

//         case RM_C610:
//             canid = C610_RX_ID(id);
//             break;

//         case RM_GM6020:
//             canid = GM6020_RX_ID(id);
//             break;
//         }

//         if (canid)
//         {
//             register_error_flag = true;
//             return false;
//         }
//     }

//     motor->id = id;
//     motor->type = type;
//     motor->next = NULL;

//     p->next = motor;

//     return true;
// }

// void RM_Motor_Transmit(void)
// {
// }
