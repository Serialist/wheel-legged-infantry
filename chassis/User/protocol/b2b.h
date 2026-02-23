/**
 * @file b2b.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef B2B_H
#define B2B_H

#include "user_lib.h"

typedef struct
{
    float vx;
    float vy;
    float vyaw;
} B2B_Chassis_Cmd_t;

void B2B_Chassis_Cmd_Encode(B2B_Chassis_Cmd_t *data, uint8_t *buf);
void B2B_Chassis_Cmd_Decode(uint8_t *buf, B2B_Chassis_Cmd_t *data);

#endif
