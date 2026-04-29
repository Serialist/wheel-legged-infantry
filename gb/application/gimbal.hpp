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

namespace vgd
{
class Gimbal
{
	enum class State
	{
		none,
		disable,
		enable_stop,
		enable_run
	};

public:
	Gimbal();
	~Gimbal();

	void Control_Loop();

	void SetPoint(float yaw, float pitch);
};
} // namespace vgd

#endif
