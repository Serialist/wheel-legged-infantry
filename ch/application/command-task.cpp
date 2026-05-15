/**
 * @file Command_Task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-21
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "command-task.hpp"
#include "Remote_Control.h"
#include "b2b.h"
#include "bsp_uart.h"
#include "chassis.hpp"
#include "cmsis_os.h"
#include "ins-task.hpp"
#include "math-utils.hpp"
#include "observer.hpp"
#include "rm_motor.h"
#include "simple-planner.h"
#include "vmc-dm.h"
#include <cmath>

using namespace vgd;

// #define RC_STOP DT7_UP
// #define RC_RUN DT7_MID
#define RC_STOP 0
#define RC_RUN 1
#define RC_JUMP 2

B2B_Chassis_Command_t ch_cmd;

extern Remote_Info_Typedef remote_ctrl;
extern Wheel_Leg_Target_t set;
extern Robo_Attitude_t att;
extern JUMP_State_t jump_state;

uint8_t prev_switch = RC_JUMP;

void Cmd_Get(void);
void Cmd_Reset(void);

// #define RC_SWITCH remote_ctrl.rc.s[DT7_SL]
#define RC_SWITCH ch_cmd.sw[0]

extern "C" void Command_Task(void const* argument) {
    /* ================================ 系统初始化 ================================ */

    rbstate = RBS_INIT;

    /* ================================ 初始化 ================================ */

    rbstate = RBS_READY;

    /* ================================ 模式切换 ================================ */
    for (;;) {
        // 停止
        if (RC_SWITCH == RC_RUN) {
            rbstate = RBS_RUN;
            Cmd_Get();
        } else if (RC_SWITCH == RC_JUMP) {
            rbstate = RBS_JUMP;
        } else {
            rbstate = RBS_STOP;
            Cmd_Reset();
        }

        prev_switch = RC_SWITCH;

        // USART_Vofa_Justfloat_Transmit(0, 0.f, 0.f);

        osDelay(1);
    }
}

// #define LEG_LEN_CMD remote_ctrl.rc.ch[DT7_RY]
// #define V_CMD remote_ctrl.rc.ch[DT7_LY]
// #define YAW_CMD remote_ctrl.rc.ch[DT7_LX]
// #define ROLL_CMD remote_ctrl.rc.ch[DT7_RX]

/// @brief 获取控制量
void Cmd_Get(void) {
    set.v = ch_cmd.ch[0];

    set.yaw = ch_cmd.ch[2];

    set.height = ch_cmd.ch[3];

    set.roll = 0;

    if (std::abs(set.v) > 0.1)
        set.x = ob.x = 0;

    set.theta = -0.04;
}

void Cmd_Reset(void) {
    set.v = 0;
    set.yaw = 0;
    set.roll = 0;
    set.x = ob.x = 0;
}
