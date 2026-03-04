/**
 * @file control.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-02
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "user_lib.h"

typedef enum
{
  RBS_NONE = 0,
  RBS_INIT,
  RBS_READY,
  RBS_ZREOFORCE,
  RBS_STOP,
  RBS_ERROR,
  RBS_EMERGENCY,
  RBS_RUNNING,
} Robo_Status_t;

typedef struct
{
  Robo_Status_t status;
} Control_t;

extern Control_t control;

#endif // CONTROL_H
