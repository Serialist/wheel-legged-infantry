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
#include "Image_Transmission.h"
#include "Remote_Control.h"
#include "b2b.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "gimbal.hpp"
#include "ins-task.hpp"
#include "math-utils.hpp"
#include "rm_motor.h"
#include "superpower.h"
#include "vmc-dm.h"

using namespace vgd;
using module::robot::Remote_Control_Mode;

extern RM_Motor_Feedback_t yaw_motor, pitch_motor;

B2B_Chassis_Command_t ch_cmd;
uint8_t buf[8];

void Command_ZeroForce(void);
void Command_Normal(void);
void Command_Jump(void);

module::robot::Robot rb;
Remote_Control_Mode rc_mode = Remote_Control_Mode::dt7;

extern "C" void Command_Task(void const* argument) {
    for (;;) {
        Remote_Message_Moniter(&remote_ctrl);

        if (remote_ctrl.rc_lost == true || // 遥控器断线
            remote_ctrl.rc.s[DT7_SL] == DT7_UP)
        {
            Command_ZeroForce();
        } else if (remote_ctrl.rc.s[DT7_SL] == DT7_MID) {
            Command_Normal();
        } else if (remote_ctrl.rc.s[DT7_SL] == DT7_DOWN) {
            Command_Jump();
        }

        /* 设置 */

        B2B_Chassis_Command_Encode(&ch_cmd, buf);

        BSP_CAN_Transmit(BSP_PORT2, B2B_CHASSIS_CMD_ID, buf);

        osDelay(1);
    }
}

void Command_ZeroForce(void) {
    gb_mode = module::Gimbal_Mode::zero_force;
    rb.cmd.enable = false;

    ch_cmd = {};

    rb.att.yaw = ins.Yaw_Angle;
    rb.att.pitch = ins.Pitch_Angle;
}

float atg1 = 0, atg2 = 0;
float yaw_angle, chassis_angle, bias_angle, chassis_yaw_setpoint;

void Command_Normal(void) {
    float setv;

    gb_mode = module::Gimbal_Mode::running;
    rb.cmd.enable = true;

    switch (rc_mode) {
            /* DT7 遥控器控制 */
        case Remote_Control_Mode::dt7:
            rb.att.vx = math::Remapf(remote_ctrl.rc.ch[DT7_LY], -660, 660, -2, 2);  // 底盘x速度
            rb.att.vy = math::Remapf(-remote_ctrl.rc.ch[DT7_LX], -660, 660, -2, 2); // 底盘y速度
            rb.att.z =
                math::Remapf(std::fmaxf(remote_ctrl.rc.ch[DT7_WHEEL], 60), 60, 660, 0.1, 0.4); // 腿
            rb.att.vyaw = -remote_ctrl.rc.ch[DT7_RX] * 0.000007;  // 底盘yaw角速度
            rb.att.vpitch = remote_ctrl.rc.ch[DT7_RY] * 0.000007; // 底盘pitch角速度
            rb.att.yaw = math::LoopClampf(rb.att.yaw + rb.att.vyaw, 0, math::two_pi); // yaw角
            rb.att.pitch =
                math::ClampAbsf(rb.att.pitch + rb.att.vpitch, math::Deg2Rad(25)); // pitch角
            break;

            /* 图传遥控器控制 */
        case Remote_Control_Mode::vt13:
            // ch_cmd.vx = VT13_Info.RC.Channel[0];
            // ch_cmd.vy = VT13_Info.RC.Channel[1];
            // ch_cmd.vyaw = VT13_Info.RC.Channel[2];
            // ch_cmd.sw[0] = VT13_Info.RC.Switch;
            // ch_cmd.button[0] = VT13_Info.RC.Left;
            // ch_cmd.button[1] = VT13_Info.RC.Right;
            // ch_cmd.button[2] = VT13_Info.RC.Stop;
            // ch_cmd.button[3] = VT13_Info.RC.Trigger;
            break;

            /* 图传键鼠控制 */
        case Remote_Control_Mode::vt13_pc:
            setv = VT13_Info.Key.Set.SHIFT ? 2.5 : 1.5;
            rb.att.vx = (VT13_Info.Key.Set.W - VT13_Info.Key.Set.S) * setv;
            rb.att.vy = (VT13_Info.Key.Set.A - VT13_Info.Key.Set.D) * setv;
            break;

        default:
            rb.att = {}; // 清零
            break;
    }

    yaw_position = rb.att.yaw;
    pitch_position = rb.att.pitch;

    // 底盘相对云台角度
    yaw_angle = -math::CircleNearestDistance(
        RM_MOTOR_ANGLE(yaw_motor.encoder),
        RM_MOTOR_ANGLE(YAW_ENCODER_OFFSET)
    );

    // 底盘对地角度
    chassis_angle = math::CircleClamp(ins.Yaw_Angle + yaw_angle);

    chassis_yaw_setpoint = math::CircleClamp(
        (std::fabsf(yaw_angle) < math::half_pi) ? (rb.att.yaw) : (rb.att.yaw + math::pi)
    );

    bias_angle = std::atan2f(rb.att.vy, rb.att.vx);
    bias_angle = bias_angle > math::half_pi ? bias_angle - math::pi : bias_angle;
    bias_angle = bias_angle < -math::half_pi ? bias_angle + math::pi : bias_angle;

    ch_cmd.ch[0] = (rb.att.vx < 0 ? -1 : 1)
        * math::ClampAbsf((math::Sqrt(rb.att.vx * rb.att.vx + rb.att.vy * rb.att.vy)), 2);
    ch_cmd.ch[1] = rb.att.vy;
    ch_cmd.ch[2] = math::CircleNearestDistance(
        chassis_angle,
        math::CircleClamp(
            chassis_yaw_setpoint // 整体
            + bias_angle
        ) // 偏走
    );

    ch_cmd.ch[3] = rb.att.z;
    ch_cmd.sw[0] = 1;
    ch_cmd.btn[0] = rb.cmd.enable;
}

void Command_Jump(void) {
    Command_Normal();
    ch_cmd.sw[0] = 2;
}
