/**
 * @file superpower.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-29
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef SUPERPOWER_H
#define SUPERPOWER_H

#include "user_lib.h"

#define SUPERPOWER_FDB_ID 0x211
#define SUPERPOWER_CMD_ID 0x210

typedef struct
{
    float chassis;

    float cap_tar;
    float referee;
    float cap;
} SuperPower_Fdb_t;

typedef float SuperPower_Cmd_t;

#endif
