/**
 * @file shoot.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-26
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 * oooooo     oooo   .oooooo.    oooooooooo.
 *  `888.     .8'   d8P'  `Y8b   `888'   `Y8b
 *   `888.   .8'   888            888      888
 *    `888. .8'    888            888      888
 *     `888.8'     888     ooooo  888      888
 *      `888'      `88.    .88'   888     d88'
 *       `8'        `Y8bood8P'   o888bood8P'
 * 
*/

/* ================================================================ include ================================================================ */

#include "shoot.hpp"
#include "cmsis_os.h"
#include "heat-limit.h"
#include "rm_motor.h"
#include "robo-config.h"
#include "simple-filter.hpp"
#include "vgd-pid.hpp"

using namespace vgd;

/* ================================================================ micro ================================================================ */

#define FEED_DIAL_GEAR_RATIO (5 / 2) // 拨盘减速比
#define FEED_DIAL_BULLET_CAPACITY 10 // 一个拨盘的弹量

/* ================================================================ variable ================================================================ */

RM_Motor_Feedback_t feed_motor, fr_motor[2];

controller::PID feed_pid = { 0, 0, 0, 0, 0 };                 // 拨盘
controller::PID fr_pid[2] = { { 15, 0.01, 0, 10000, 1000 },   // 0 left
                              { 15, 0.01, 0, 10000, 1000 } }; // 1 right

float fr_velocity = FR_DEFAULT_VELOCITY, // 摩擦轮射速
    feed_freq = 0, feed_velocity = 0,    // 拨盘
    fr_current[2], feed_current;         // 发送电流

float test[2] = { 0 };

/* ================================================================ prototype ================================================================ */

/* ================================================================ function ================================================================ */

extern "C" void Shoot_Task(void const* argument) {
    for (;;) {
        /* ================ 驱动摩擦轮 ================ */
        fr_current[LEFT] = fr_pid[LEFT].Update(-fr_velocity, fr_motor[LEFT].velocity);
        fr_current[RIGHT] = fr_pid[RIGHT].Update(fr_velocity, fr_motor[RIGHT].velocity);

        test[0] = -fr_motor[LEFT].velocity;

        RM_Motor_Control_Transmit(
            BSP_PORT1,
            M3508_TX_ID_1,
            (RM_Motor_Control_t) { (int16_t)fr_current[LEFT], (int16_t)fr_current[RIGHT], 0, 0 }
        );

        /* ================ 驱动拨盘 ================ */
        feed_velocity = // 单位rpm
            feed_freq * 60 / FEED_DIAL_BULLET_CAPACITY / FEED_DIAL_GEAR_RATIO * M2006_GEAR_RATIO;

        feed_current = feed_pid.Update(feed_velocity, feed_motor.velocity);

        osDelay(1);
    }
}
