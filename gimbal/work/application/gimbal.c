/**
 * @file gimbal.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-18
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "gimbal.h"
#include "user_lib.h"

float target_yaw = 0;
float target_pitch = 0;
// float target_vyaw = 0;
// float target_vpitch = 0;

float current_yaw = 0;
float current_pitch = 0;

float control_current_yaw = 0;
float control_current_pitch = 0;

void gimbal_init(void)
{
    yaw.init();
    pitch.init();
}

void gimbal_task(void)
{
    [ target_yaw, target_pitch ] = control.get();
    [ current_yaw, current_pitch ] = ins.get();

    control_current_yaw = yaw_pid.update(target_yaw, current_yaw);
    control_current_pitch = pitch_pid.update(target_pitch, current_pitch);

    yaw.tx(control_current_yaw, control_current_pitch, 0, 0);
}
