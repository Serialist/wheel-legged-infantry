/**
 * @file upper_lower_comm.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-03
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef UPPER_LOWER_COMM_H
#define UPPER_LOWER_COMM_H

#include "user_lib.h"

typedef struct
{
    float epitch;
    float eyaw;
    float ex;
    bool fire;
} ULComm_Aimbot_Cmd_t;

void ULComm_Aimbot_Cmd_Encode(ULComm_Aimbot_Cmd_t *data, uint8_t *buf);
bool ULComm_Aimbot_Cmd_Decode(uint8_t *buf, ULComm_Aimbot_Cmd_t *data);

#endif
