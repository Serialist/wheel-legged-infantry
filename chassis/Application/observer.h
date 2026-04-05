/**
 * @file observer.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2026-01-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef __TEST_TASK_H
#define __TEST_TASK_H

/**************************************************************** include ****************************************************************/

#include "user_lib.h"
#include "stdint.h"
#include "INS_task.h"
#include "wheel_legged_chassis.h"
#include "chassismotor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "vmc-dm.h"

/**************************************************************** define ****************************************************************/

/**************************************************************** struct ****************************************************************/

typedef struct
{
    float x;
    float v;
} Observer_t;

/**************************************************************** extren ****************************************************************/

extern Observer_t ob;

/**************************************************************** proto ****************************************************************/

#endif
