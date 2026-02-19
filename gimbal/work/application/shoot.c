/**
 * @file shoot.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-18
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "shoot.h"
#include "user_lib.h"

float friction_speed1;

void Shoot_Init(void)
{
    pid.init();
    friction.init();
    feeddial.init();
}

void Shoot_Task()
{
    friction_speed(friction)
}
