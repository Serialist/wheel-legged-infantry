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
    // 摩擦轮模式
    enum class FrictionMode {
        Disable,
        Enable,
    };

    // 拨盘模式（发射模式）
    enum class FeedMode {
        Disable, // 无力
        Single,  // 单发
        Auto,    // 自动
        Unload,  // 退弹
    };

    FrictionMode fricMode;
    FeedMode fdMode;

    Shoot() = default;

    /// @brief 开火扳机，true 开火
    void Fire(bool trigger);
};

} // namespace module
} // namespace rb2

extern rb2::module::Shoot shoot;

#endif
