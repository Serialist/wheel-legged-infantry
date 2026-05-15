/**
 * @file Command_Task.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-21
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef COMMAND_TASK_H
#define COMMAND_TASK_H

#include <stdint.h>

namespace vgd {
namespace module {
namespace robot {

enum class Remote_Control_Mode {
    none,
    serial, // 串口发送
    dt7,    // dt7 遥控器
    dt7_pc, // dt7 遥控器的键鼠
    vt13,   // vt13 图传遥控器
    vt13_pc // 图传键鼠
};

struct Attitude {
    float x, y, z;
    float vx, vy, vz;
    float ax, ay, az;

    float roll, pitch, yaw;
    float vroll, vpitch, vyaw;
    float aroll, apitch, ayaw;

    std::int32_t nroll, npitch, nyaw; // 套圈计数
    double troll, tpitch, tyaw;       // 任意角
};

enum class Status { Idle, Init, Stop, Run, Error };

struct Command {
    bool enable = false;  // 启动
    bool reboot = false;  // 重启
    bool sp = false;      // super power 超电
    bool spinbot = false; // 小陀螺
    bool fire = false;    // 开火
    bool jump = false;    // 跳
    bool unstuck = false; // 脱困
};

class Robot {
public:
    Attitude att;
    Status status;
    Command cmd;
};

} // namespace robot
} // namespace module
} // namespace vgd

#endif // CONTROL_TASK_H
