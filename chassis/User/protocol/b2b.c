/**
 * @file b2b.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "b2b.h"

#define SCALE_FACTOR 100.0f

/**
 * @brief 底盘控制命令编码
 *
 * @param vx
 * @param vy
 * @param vyaw
 * @param buf 输出 8 字节
 */

void B2B_Chassis_Cmd_Encode(B2B_Chassis_Cmd_t *data, uint8_t *buf)
{

    int16_t vx_int = (int16_t)roundf(data->vx * SCALE_FACTOR);
    int16_t vy_int = (int16_t)roundf(data->vy * SCALE_FACTOR);
    int16_t vyaw_int = (int16_t)roundf(data->vyaw * SCALE_FACTOR);

    buf[0] = (vx_int >> 8) & 0xFF;
    buf[1] = vx_int & 0xFF;
    buf[2] = (vy_int >> 8) & 0xFF;
    buf[3] = vy_int & 0xFF;
    buf[4] = (vyaw_int >> 8) & 0xFF;
    buf[5] = vyaw_int & 0xFF;
}

/**
 * @brief 底盘控制命令解码
 *
 * @param buf
 * @param vx
 * @param vy
 * @param vyaw
 */
void B2B_Chassis_Cmd_Decode(uint8_t *buf, B2B_Chassis_Cmd_t *data)
{

    int16_t vx_int = (buf[0] << 8) | buf[1];
    int16_t vy_int = (buf[2] << 8) | buf[3];
    int16_t vyaw_int = (buf[4] << 8) | buf[5];

    data->vx = (float)vx_int / SCALE_FACTOR;
    data->vy = (float)vy_int / SCALE_FACTOR;
    data->vyaw = (float)vyaw_int / SCALE_FACTOR;
}

#undef SCALE_FACTOR

// Command_Encode(Command_t *self, uint8_t *buf)
// {
// 	buf[0] = (self->move << 4) |
// 			 (self->rot & 0x0F);
// 	buf[1] = (self->leg << 4) |
// 			 (self->shoot & 0x0F);
// }

// Command_Decode(Command_t *self, uint8_t *buf)
// {
// 	self->move = buf[0] >> 4;
// 	self->rot = buf[0] & 0x0F;
// 	self->leg = buf[1] >> 4;
// 	self->shoot = buf[1] & 0x0F;
// }
