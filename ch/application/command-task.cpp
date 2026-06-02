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
#include "bsp-adapter.h"
#include "bsp_uart.h"
#include "chassis.hpp"
#include "cmsis_os.h"
#include "cubemars-motor.h"
#include "ins-task.hpp"
#include "math-utils.hpp"
#include "observer.hpp"
#include "rm_motor.h"
#include "simple-planner.h"
#include "superpower.h"
#include "vmc-dm.h"
#include "wheelleg-power-manager.hpp"
#include <cmath>

using namespace rb2;

namespace pwctrl = rb2::module::wl_pwctrl;

B2B_Chassis_Command_t ch_cmd;
extern SuperPower_Fdb_t sp_fdb;
extern RM_Motor_Feedback_t m3508[2];
extern Motor_AK_RxData_t ak10[4];

extern Remote_Info_Typedef remote_ctrl;
extern Wheel_Leg_Target_t set;
extern Robo_Attitude_t att;
extern JUMP_State_t jump_state;

uint8_t prev_switch = 0;

void Cmd_Get(void);
void Cmd_Reset(void);

bool motor_offline[2] = { false, false };

extern "C" void Command_Task(void const* argument) {
    /* ================================ 系统初始化 ================================ */

    rbstate = RBS_INIT;

    /* ================================ 初始化 ================================ */

    rbstate = RBS_READY;
    set.theta = -0.04;

    /* ================================ 模式切换 ================================ */
    for (;;) {
        /* 监测任务 */

        Remote_Message_Moniter(&remote_ctrl);
        AK_Motor_Monitor(&ak10[0]);
        AK_Motor_Monitor(&ak10[1]);
        AK_Motor_Monitor(&ak10[2]);
        AK_Motor_Monitor(&ak10[3]);
        RM_MOTOR_MONITOR(&m3508[LEFT]);
        RM_MOTOR_MONITOR(&m3508[RIGHT]);
        B2B_MONITOR(&ch_cmd);
        SuperPower_Monitor(&sp_fdb);

        motor_offline[LEFT] = RM_MOTOR_IS_OFFLINE(&m3508[LEFT]);
        motor_offline[RIGHT] = RM_MOTOR_IS_OFFLINE(&m3508[RIGHT]);

        if (ch_cmd.sw[0] == 0         // 停止
            || ch_cmd.btn[0] == false // 使能按钮
            || RM_MOTOR_IS_OFFLINE(&m3508[LEFT]) || RM_MOTOR_IS_OFFLINE(&m3508[RIGHT])
            || B2B_IS_OFFLINE(&ch_cmd))
        {
            rbstate = RBS_STOP;
            Cmd_Reset();

        } else if (ch_cmd.sw[0] == 1) {
            rbstate = RBS_RUN;
            Cmd_Get();
        } else {
            rbstate = RBS_STOP;
            Cmd_Reset();
        }
        /* else if (RC_SWITCH == 2) {
            rbstate = RBS_JUMP;
            Cmd_Get();
        }  */

        prev_switch = ch_cmd.sw[0];

        // USART_Vofa_Justfloat_Transmit(0, 0.f, 0.f);

        osDelay(1);
    }
}

/// @brief 获取控制量
void Cmd_Get(void) {
    set.v = ch_cmd.ch[0];

    set.yaw = ch_cmd.ch[2];

    set.height = ch_cmd.ch[3];

    rbflag.enable = ch_cmd.btn[0];
    rbflag.sp = ch_cmd.btn[1];
    rbflag.spinbot = ch_cmd.btn[2];

    // pwctrl::setMode(ch_cmd.btn[1] == true ? pwctrl::MODE::CAP : pwctrl::MODE::BATTERY);
    pwctrl::setMode(pwctrl::MODE::BATTERY);

    if (std::abs(set.v) > 0.1)
        set.x = ob.x = 0;
}

void Cmd_Reset(void) {
    set.v = 0;
    set.yaw = 0;
    set.x = ob.x = 0;
    rbflag.enable = false;
    pwctrl::setMode(pwctrl::MODE::BATTERY);
}
