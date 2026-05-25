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

#ifndef SHOOT_HPP
#define SHOOT_HPP

#include "stdint.h"

namespace rb2 {
namespace module {

class Shoot {
public:
    enum class State {
        SafetyOn,  // 有保险（使能，电机给力但不动）
        SafetyOff, // 开保险（开摩擦轮）
    };

    enum class Mode {
        Disable, // 无力
        Single,  // 单发（一颗弹丸）
        // Burst,   // 连发（点射，连续射 n 发）
        Auto, // 自动（一直发射）
        // Semi,   // 半自动，自动切换单发和自动
        Unload, // 退弹
    };

    Shoot() = default;

    /// @brief 触发开火（扳机）true  开火
    void Fire(bool trigger);
    /// @brief 选择模式（快慢机）
    inline void Selector(Mode mode) {
        this->mode = mode;
    };
    /// @brief 状态控制
    inline void SetState(State state) {
        this->state = state;
    }
    inline Mode GetMode(void) {
        return mode;
    }
    inline State GetState(void) {
        return state;
    }

private:
    State state;
    Mode mode;
    uint32_t count;
};

} // namespace module
} // namespace rb2

extern rb2::module::Shoot shoot;

#endif
