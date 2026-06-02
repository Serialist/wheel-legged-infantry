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
#include "math-adapter.hpp"
#include "math-utils.hpp"
#include "rm_motor.h"
#include "robo-config.h"
#include "shoot.hpp"
#include "superpower.h"
#include "vmc-dm.h"

using namespace rb2;
using module::Gimbal;
using module::Shoot;
using module::robot::Remote_Control_Mode;

extern RM_Motor_Feedback_t yaw_motor, pitch_motor, feed_motor, fr_motor[2];

B2B_Chassis_Command_t ch_cmd;
uint8_t buf[8];

void Command_ZeroForce(void);
void Command_Normal(void);
void Command_Jump(void);

module::robot::Robot rb;
Remote_Control_Mode rc_mode = Remote_Control_Mode::vt13;

extern "C" void Command_Task(void const* argument) {
    rb.att.vx = 0;
    rb.att.vy = 0;
    rb.att.z = 0.1;
    rb.att.vyaw = 0;
    rb.att.vpitch = 0;
    rb.att.yaw = ins.Yaw_Angle;
    rb.att.pitch = ins.Pitch_Angle;
    rb.cmd.ch_enable = false;
    rb.cmd.spinbot = false;
    gimbal.mode = Gimbal::Mode::zero_force;
    shoot.fricMode = Shoot::FrictionMode::Disable;
    shoot.fdMode = Shoot::FeedMode::Disable;

    for (;;) {
        Remote_Message_Moniter(&remote_ctrl);
        VT13_Monitor(&VT13_Info);
        RM_MOTOR_MONITOR(&yaw_motor);
        RM_MOTOR_MONITOR(&pitch_motor);
        RM_MOTOR_MONITOR(&feed_motor);
        RM_MOTOR_MONITOR(&fr_motor[0]);
        RM_MOTOR_MONITOR(&fr_motor[1]);

        if (VT13_Info.RC.Switch == 0) {
            if (remote_ctrl.rc_lost == true || remote_ctrl.rc.s[DT7_SL] == DT7_UP)
                Command_ZeroForce();
            else {
                rc_mode = Remote_Control_Mode::dt7;
                Command_Normal();
            }
        } else if (VT13_Info.RC.Switch == 1) {
            rc_mode = Remote_Control_Mode::vt13_pc;
            Command_Normal();
        }

        /* 设置 */

        B2B_Chassis_Command_Encode(&ch_cmd, buf);

        BSP_CAN_Transmit(BSP_PORT2, B2B_CHASSIS_CMD_ID, buf);

        osDelay(1);
    }
}

void Command_ZeroForce(void) {
    gimbal.mode = Gimbal::Mode::zero_force;
    rb.cmd.gb_enable = false;
    rb.cmd.ch_enable = false;

    ch_cmd = {};

    rb.att.yaw = ins.Yaw_Angle;
    rb.att.pitch = ins.Pitch_Angle;

    shoot.fdMode = Shoot::FeedMode::Disable;
    shoot.fricMode = Shoot::FrictionMode::Disable;
    shoot.Fire(false);
}

float atg1 = 0, atg2 = 0;
float yaw_angle, chassis_angle, bias_angle, chassis_yaw_setpoint;
bool re0 = false; // 头尾

float mouse_ratio = 0.00005;

void Command_Normal(void) {
    float setv;

    switch (rc_mode) {
            /* DT7 遥控器控制 */
        case Remote_Control_Mode::dt7:
            switch (remote_ctrl.rc.s[DT7_SL]) {
                case DT7_UP:
                    rb.cmd.ch_enable = false;
                    rb.cmd.spinbot = false;

                    gimbal.mode = Gimbal::Mode::zero_force;

                    shoot.fricMode = Shoot::FrictionMode::Disable;
                    shoot.fdMode = Shoot::FeedMode::Disable;
                    shoot.Fire(false);
                    break;
                case DT7_MID:
                    rb.att.vx =
                        math::Remapf(remote_ctrl.rc.ch[DT7_LY], -660, 660, -2, 2); // 底盘x速度
                    rb.att.vy =
                        math::Remapf(-remote_ctrl.rc.ch[DT7_LX], -660, 660, -2, 2); // 底盘y速度
                    rb.att.z = remote_ctrl.rc.s[DT7_SR] == DT7_UP ? 0.1
                        : remote_ctrl.rc.s[DT7_SR] == DT7_MID     ? 0.25
                        : remote_ctrl.rc.s[DT7_SR] == DT7_DOWN    ? 0.25
                                                                  : 0.1; // 腿

                    rb.att.vyaw = -remote_ctrl.rc.ch[DT7_RX] * 0.007;  // yaw角速度
                    rb.att.vpitch = remote_ctrl.rc.ch[DT7_RY] * 0.007; // pitch角速度
                    rb.att.yaw = math::LoopClampf(rb.att.yaw + rb.att.vyaw * 1e-3, 0, math::two_pi);
                    rb.att.pitch =
                        math::ClampAbsf(rb.att.pitch + rb.att.vpitch * 1e-3, math::Deg2Rad(25));

                    rb.cmd.ch_enable = true;
                    rb.cmd.spinbot = remote_ctrl.rc.s[DT7_SR] == DT7_DOWN ? true : false;

                    gimbal.mode = Gimbal::Mode::running;

                    shoot.fricMode = Shoot::FrictionMode::Disable;
                    shoot.fdMode = Shoot::FeedMode::Disable;

                    break;
                case DT7_DOWN:
                    rb.att.vyaw = -remote_ctrl.rc.ch[DT7_RX] * 0.007;  // yaw角速度
                    rb.att.vpitch = remote_ctrl.rc.ch[DT7_RY] * 0.007; // pitch角速度
                    rb.att.yaw = math::LoopClampf(rb.att.yaw + rb.att.vyaw * 1e-3, 0, math::two_pi);
                    rb.att.pitch =
                        math::ClampAbsf(rb.att.pitch + rb.att.vpitch * 1e-3, math::Deg2Rad(25));

                    rb.cmd.ch_enable = false;

                    gimbal.mode = Gimbal::Mode::zero_force;

                    shoot.fricMode = remote_ctrl.rc.s[DT7_SR] != DT7_UP
                        ? Shoot::FrictionMode::Enable
                        : Shoot::FrictionMode::Disable;
                    shoot.fdMode = math::Deadzonef(remote_ctrl.rc.ch[DT7_WHEEL], 0, 10) == 0
                        ? Shoot::FeedMode::Auto
                        : Shoot::FeedMode::Unload;
                    shoot.Fire(remote_ctrl.rc.s[DT7_SR] == DT7_DOWN);
                    break;
                default:
                    break;
            }
            break;

            /* 图传遥控器控制 */
            // case Remote_Control_Mode::vt13:
            //     rb.att.vx = math::Remapf(VT13_Info.RC.Channel[DT7_LX], -660, 660, -2, 2);  // 底盘x速度
            //     rb.att.vy = math::Remapf(-VT13_Info.RC.Channel[DT7_LY], -660, 660, -2, 2); // 底盘y速度
            //     rb.att.z = math::Remapf(std::fmaxf(VT13_Info.RC.Wheel, 60), 60, 660, 0.1, 0.4); // 腿
            //     rb.att.vyaw = -VT13_Info.RC.Channel[DT7_RX] * 0.007;  // 底盘yaw角速度
            //     rb.att.vpitch = VT13_Info.RC.Channel[DT7_RY] * 0.007; // 底盘pitch角速度
            //     rb.att.yaw = math::LoopClampf(rb.att.yaw + rb.att.vyaw * 1e-3, 0, math::two_pi);
            //     rb.att.pitch = math::ClampAbsf(rb.att.pitch + rb.att.vpitch * 1e-3, math::Deg2Rad(25));

            //     rb.cmd.ch_enable = true;
            //     rb.cmd.spinbot = false;

            //     shoot.fricMode = Shoot::FrictionMode::Disable;
            //     shoot.fdMode = Shoot::FeedMode::Disable;
            //     shoot.Fire(false);

            //     gimbal.mode = Gimbal::Mode::running;
            //     break;

            /* 图传键鼠控制 */
        case Remote_Control_Mode::vt13_pc:

            setv = VT13_Info.Key.Set.SHIFT ? 2 : 1;
            rb.att.vx = (VT13_Info.Key.Set.W - VT13_Info.Key.Set.S) * setv;
            rb.att.vy = (VT13_Info.Key.Set.A - VT13_Info.Key.Set.D) * setv;
            // rb.att.vz = VT13_Info.Mouse.Z * mouse_ratio;
            // rb.att.z = math::Clampf(rb.att.z + rb.att.vz, 0.1, 0.4);
            { // 按 C 切换高度
                static bool prev_c = false;
                static uint8_t height_mode = 0;

                if (prev_c == false && VT13_Info.Key.Set.C == true) {
                    height_mode = (height_mode + 1) % 3;
                }
                rb.att.z = height_mode == 0 ? 0.1
                    : height_mode == 1      ? 0.25
                    : height_mode == 2      ? 0.45
                                            : 0.1; // 腿
                prev_c = VT13_Info.Key.Set.C;
            }
            rb.att.vyaw = -VT13_Info.Mouse.X * mouse_ratio;   // 底盘yaw角速度
            rb.att.vpitch = -VT13_Info.Mouse.Y * mouse_ratio; // 底盘pitch角速度
            rb.att.yaw = math::LoopClampf(rb.att.yaw + rb.att.vyaw, 0, math::two_pi); // yaw角
            rb.att.pitch =
                math::ClampAbsf(rb.att.pitch + rb.att.vpitch, math::Deg2Rad(25)); // pitch角

            // rb.cmd.sp = VT13_Info.Key.Set.SHIFT ? true : false;

            { // 按下 Z 切换无力
                static bool prev_z = false;
                static bool zero_force_mode = true;

                if (prev_z == false && VT13_Info.Key.Set.Z == true) {
                    zero_force_mode = !zero_force_mode;
                }

                if (zero_force_mode) {
                    rb.cmd.ch_enable = false;
                    gimbal.mode = Gimbal::Mode::zero_force;
                } else {
                    rb.cmd.ch_enable = true;
                    gimbal.mode = Gimbal::Mode::running;
                }
                prev_z = VT13_Info.Key.Set.Z;
            }

            // rb.cmd.ch_enable = true;
            // gimbal.mode = Gimbal::Mode::running;

            { // 按下Q切换小陀螺状态
                static bool prev_q = false;
                if (prev_q == false && VT13_Info.Key.Set.Q == true) {
                    rb.cmd.spinbot = !rb.cmd.spinbot;
                }
                prev_q = VT13_Info.Key.Set.Q;
            }

            { // 按下F切换摩擦轮状态
                static bool prev_f = false;
                if (prev_f == false && VT13_Info.Key.Set.F == true) {
                    if (shoot.fricMode == Shoot::FrictionMode::Disable) {
                        shoot.fricMode = Shoot::FrictionMode::Enable;

                    } else if (shoot.fricMode == Shoot::FrictionMode::Enable) {
                        shoot.fricMode = Shoot::FrictionMode::Disable;
                    }
                }
                prev_f = VT13_Info.Key.Set.F;
            }

            // 按下退弹，默认连射
            shoot.fdMode = VT13_Info.Key.Set.R ? Shoot::FeedMode::Unload : Shoot::FeedMode::Auto;
            // 按下开火
            shoot.Fire(VT13_Info.Mouse.Press_L ? true : false);

            break;

        default:
            rb.att = {}; // 清零

            rb.cmd.ch_enable = false;

            gimbal.mode = Gimbal::Mode::zero_force;

            shoot.fricMode = Shoot::FrictionMode::Disable;
            shoot.fdMode = Shoot::FeedMode::Disable;
            shoot.Fire(false);
            break;
    }

    gimbal.yaw_position = rb.att.yaw;
    gimbal.pitch_position = rb.att.pitch;
    gimbal.yaw_velocity_feedforward = rb.cmd.spinbot ? (-math::two_pi) : 0;

    // 底盘相对云台角度
    yaw_angle = -math::CircleNearestDistance(
        RM_MOTOR_ANGLE(yaw_motor.encoder),
        RM_MOTOR_ANGLE(YAW_ENCODER_OFFSET)
    );

    // 底盘对地角度
    chassis_angle = math::CircleClamp(ins.Yaw_Angle + yaw_angle);

    bias_angle = std::atan2f(rb.att.vy, rb.att.vx);
    bias_angle = bias_angle > math::half_pi ? bias_angle - math::pi : bias_angle;
    bias_angle = bias_angle < -math::half_pi ? bias_angle + math::pi : bias_angle;

    chassis_yaw_setpoint = rb.att.yaw + bias_angle;
    re0 = std::fabsf(math::CircleNearestDistance(chassis_angle, chassis_yaw_setpoint))
        < math::half_pi;
    chassis_yaw_setpoint =
        math::CircleClamp(re0 ? chassis_yaw_setpoint : chassis_yaw_setpoint + math::pi);

    ch_cmd.ch[0] = (((rb.att.vx < 0) == re0) ? -1 : 1)
        * math::ClampAbsf((math::Sqrt(rb.att.vx * rb.att.vx + rb.att.vy * rb.att.vy)), 2);
    ch_cmd.ch[1] = rb.att.vy;
    ch_cmd.ch[2] = math::CircleNearestDistance(chassis_angle, chassis_yaw_setpoint);

    ch_cmd.ch[3] = rb.att.z;
    ch_cmd.sw[0] = 1;
    ch_cmd.btn[0] = rb.cmd.ch_enable;
    ch_cmd.btn[1] = rb.cmd.sp;
    ch_cmd.btn[2] = rb.cmd.spinbot;
}
