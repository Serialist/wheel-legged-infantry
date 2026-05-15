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

#include "stdint.h"

namespace vgd {
namespace module {
class Shoot {
private:
    uint32_t count;

public:
    enum class Type {
        None,
        Single, // 单发（一颗弹丸）
        Burst,  // 连发（点射，连续射 n 发）
        Auto,   // 自动（一直发射）
        Semi,   // 半自动，自动切换单发和自动
        Max
    } type;

    enum class State {
        None,
        Disable,          // 无力
        Enable_SafetyOn,  // 有保险（使能，电机给力但不动）
        Enable_SafetyOff, // 开保险（开摩擦轮）
    } state;

    enum class Mode {
        None,
        Disable,       // 无力
        Enable_Stop,   // 停
        Enable_Fire,   // 开火
        Enable_Unload, // 退弹
    };

    Shoot();
    void Control_Loop();

    /// @brief 触发开火（扳机）true  开火
    void Fire(bool trigger);
    /// @brief 选择模式（快慢机）
    inline void Selector(Type type) {
        this->type = type;
    };
    /// @brief 状态控制
    void SetState(State state);
};

} // namespace module
} // namespace vgd

#endif
