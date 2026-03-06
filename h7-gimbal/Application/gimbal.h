/**
 * @file gimbal.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "user_lib.h"

typedef struct
{

  struct
  {

    float yaw;
    float pitch;

  } target;

  struct
  {

    float yaw;
    float pitch;

  } feedback;

  struct
  {

    int16_t yaw;
    int16_t pitch;

  } output;

} Gimbal_t;

extern Gimbal_t gimbal;

extern void (*gimbal_control)(void);

void Gimbal_ZeroForce(void);
void Gimbal_Running(void);

#endif // CONTROL_TASK_H