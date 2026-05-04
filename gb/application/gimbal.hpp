/**
 * @file gimbal.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-26
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef GIMBAL_H
#define GIMBAL_H

#include "utils.h"

extern float yaw_position, // 角度环 目标
	yaw_velocity,		   // 速度环 目标
	pitch_position,		   // 角度环 目标
	pitch_velocity;		   // 速度环 目标

enum class Gimbal_Mode
{
	zero_force,
	running
};

extern Gimbal_Mode gb_mode;

#endif
