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

namespace rb2 {
namespace module {

class Gimbal {
public:
    enum class Mode { zero_force, running };

    Gimbal() = default;

    float yaw_position, // 角度环 目标
        yaw_velocity,   // 速度环 目标
        pitch_position, // 角度环 目标
        pitch_velocity; // 速度环 目标

    float // 用于小陀螺，旋转时的速度前馈，实际是底盘的速度。因为云台yaw输出的速度实际上是云台相对于底盘的速度，所以要变换一下。
        yaw_velocity_feedforward;
    Mode mode;
};

} // namespace module
} // namespace rb2

extern rb2::module::Gimbal gimbal;

extern float yaw_position, // 角度环 目标
    yaw_velocity,          // 速度环 目标
    pitch_position,        // 角度环 目标
    pitch_velocity;        // 速度环 目标

#endif
