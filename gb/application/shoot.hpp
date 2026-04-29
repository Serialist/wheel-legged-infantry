/**
 * @file shoot.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-28
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef GIMBAL_H
#define GIMBAL_H

#include "utils.h"

namespace vgd
{
class Shoot
{
private:
	uint32_t count;

public:
	enum class Type
	{
		None,
		Single, // 单发（一颗弹丸）
		Burst,	// 连发（点射，连续射 n 发）
		Auto,	// 自动（一直发射）
		Semi,	// 半自动，自动切换单发和自动
		Max
	};

	enum class State
	{
		None,
		Disable,				 // 无力
		Enable_SafetyOn,		 // 上保险（使能，电机给力但不动）
		Enable_SafetyOn_Unload,	 // 退弹
		Enable_SafetyOn_Fire,	 // 开火（此时摩擦轮不开）
		Enable_SafetyOff,		 // 开保险（开摩擦轮）
		Enable_SafetyOff_Unload, // 退弹（射击时退弹）
		Enable_SafetyOff_Fire,	 // 开火
		Max
	};

	Shoot();
	void Control_Loop();

	/// @brief 触发开火
	/// @param trigger 扳机 false - 按下 true - 弹起
	void Fire(bool trigger);

	void SetType(Type type);
	void SetState(State state);
};
} // namespace vgd

#endif
