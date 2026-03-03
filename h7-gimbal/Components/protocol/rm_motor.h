/**
 * @file rm_motor.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-01
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef RM_MOTOR_H
#define RM_MOTOR_H

/* ================================================================ include ================================================================ */

#include "user_lib.h"

/* ================================================================ define ================================================================ */

// C620 电调，适用 M3508 电机
#define C620_TX_ID_1 0x200
#define C620_TX_ID_2 0x1FF
#define C620_RX_ID(id) (0x200 + id)
// C610 电调，适用 M2006 电机
#define C610_TX_ID_1 0x200
#define C610_TX_ID_2 0x1FF
#define C610_RX_ID(id) (0x200 + id)
// GM6020 电机
#define GM6020_TX_V_ID_1 0x1FF // 电压控制
#define GM6020_TX_V_ID_2 0x2FF
#define GM6020_TX_I_ID_1 0x1FE // 电流控制
#define GM6020_TX_I_ID_2 0x2FE
#define GM6020_RX_ID(id) (0x204 + id)

#define RM_MOTOR_ANGLE(data) ((float)(data)->angle * 2 * PI / 8191.0f) // unit: rad
#define RM_MOTOR_SPEED(data) ((float)(data)->speed)                    // unit: rpm
#define RM_MOTOR_CURRENT(data) ((data)->current * 20.f / 16384.f)      // unit: A
#define RM_MOTOR_TEMP(data) ((data)->temp)                             // unit: Celsius

// #define RM_MOTOR_SPEED(data) ((float)(data)->speed / 60.0f * 2 * PI / (268.0f / 17.0f))

#define XROLL_TORQUE(curren) ((float)curren * 0.00664267103370976f)

/* ================================================================ struct ================================================================ */

typedef struct
{
    int32_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
} RM_Motor_Fdb_t;

typedef int16_t RM_Motor_Cmd_t;

typedef enum
{
    RM_NONE = 0,
    RM_C620,
    RM_C610,
    RM_GM6020,
} RM_Motor_Type_t;

typedef struct RM_Motor
{
    uint8_t id;
    RM_Motor_Type_t type;
    int16_t cmd;
    RM_Motor_Fdb_t fdb;
} RM_Motor_t;

/* ================================================================ value ================================================================ */

/* ================================================================ proto ================================================================ */

void RM_Motor_Cmd_Encode(int16_t data1, int16_t data2, int16_t data3, int16_t data4, uint8_t *buf);
void RM_Motor_Fdb_Decode(uint8_t *buf, RM_Motor_Fdb_t *data);

#endif
