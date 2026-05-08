/***********************************************
 * @author Serialist (ba3pt@chd.edu.cn)
 * @file wheel_legged_chassis.c
 * @brief
 * @version 0.1.0
 * @date 2025-10-16
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 * oooooo     oooo   .oooooo.    oooooooooo.
 *  `888.     .8'   d8P'  `Y8b   `888'   `Y8b
 *   `888.   .8'   888            888      888
 *    `888. .8'    888            888      888
 *     `888.8'     888     ooooo  888      888
 *      `888'      `88.    .88'   888     d88'
 *       `8'        `Y8bood8P'   o888bood8P'
 *
 *************************************************/

/* ================================================================ include ================================================================ */

#include "rm_motor.h"
#include "Remote_Control.h"
#include "cmsis_os.h"
#include "cubemars_motor.h"
#include "ins-task.hpp"
#include "lqr.h"
#include "observer.hpp"
#include "ptpid.hpp"
#include "vmc-dm.h"
#include "utils.h"
#include "simple-planner.h"
#include "simple-filter.hpp"
#include "leg-spd.h"

#include "chassis.hpp"

using namespace vgd;

/* ================================================================ micro ================================================================ */

/* ================================================================ variable ================================================================ */

Motor_AK_RxData_t ak10[4];
RM_Motor_Feedback_t m3508[2];

Robo_State_t rbstate, prev_rbstate; // 机器人模式
Robo_Flag_t rbflag, prev_rbflag;	// 机器人状态
JUMP_State_t jump_state = JPS_NONE; // 跳跃状态机

PID pid_tpl(14, 0, 3, 10, 0), // 离地关节 pid
	pid_tpr(14, 0, 3, 10, 0), // 离地关节 pid

	length_pid[2]{{660, 0, -130, 80, 0}, {660, 0, -130, 80, 0}}, // 腿长 pid

	jump_length_pid[2]{{1300, 0, 60, 300, 0}, {1300, 0, 60, 300, 0}},	// 跳跃 pid
	damping_pid[2]{{1000, 0, 20000, 300, 0}, {1000, 0, 20000, 300, 0}}, // 阻尼 pid
	land_pid[2]{{500, 0, 10000, 150, 0}, {500, 0, 10000, 150, 0}},		// 落地缓冲 pid

	yaw_pid(4, 0, 1, 999, 0), vyaw_pid{0, 0, 0, 0, 0}, // yaw 旋转 pid
	roll_pid(100, 0, 10, 30, 0),					   // roll 轴补偿 pid
	tp_pid(20, 0, 2, 10, 0);						   // 劈叉 pid

algorithm::LowPass_Order_1 dtheta_filter[2] = {{15, 1000}, {15, 1000}}, // leg dtheta
	dl0_flr[2] = {{80, 1000}, {80, 1000}};								// leg dlength

Ramp_t ramp_leg_length; // 腿长斜坡

Robo_Attitude_t att;	// 机体姿态
Wheel_Leg_Target_t set; // 目标值
VMC_t leg[2];			// 腿 VMC 解算

float turn_t;	// yaw 补偿
float roll_t;	// roll 补偿
float tp_alpha; // 劈叉

// 腿前馈

float leg_ff = ROD2_MASS * GRAVITY_ACCEL / 2; // 前馈重力
float offground_leg_ff = -15.f;				  // 离地前馈
float shrink_leg_ff = -15.f;				  // 收腿前馈

// 状态量
float xl[6], xr[6];

// 控制量
float ul[2], ur[2];
float ul_temp[12], ur_temp[12];

// debug variable
float t1 = 0, t2 = 0;

/* ================================================================ prototype ================================================================ */

void Motor_Enable(void);
void Motor_Disable(void);
void Wheel_Leg_Attitude_Calc(void);
void Wheel_Leg_Control(void);
void Yaw_Control(void);
void Chassis_Motor_Transmit(void);
void Jump_FSM(void); // 正常跳
void Pmuj_FSM(void); // 磕上台阶
void Chassis_Zero(void);
void Roll_Height_Calc(
	float ll, float lr, float w, float r, float h, float *ltl, float *ltr, float min_len, float max_len);

/* ================================================================ function ================================================================ */

extern "C" void Chassis_Task(void const *argument)
{
	VMC_Init(&leg[LEFT]);
	VMC_Init(&leg[RIGHT]);

	Ramp_Init(&ramp_leg_length, .1f, -0.0008f, 0.0008f);

	set.height = 0.1f;

	Motor_Enable();

	for (;;)
	{
		/* ================ 状态更新 ================ */

		if (rbflag.reable == true)
		{
			rbflag.reable = false;
			Motor_Disable();
			Motor_Enable();
		}

		if (rbflag.enable == true && prev_rbflag.enable == false)
			Motor_Enable();
		else if (rbflag.enable == false && prev_rbflag.enable == true)
			Motor_Disable();

		Wheel_Leg_Attitude_Calc();

		/* ================ 控制 ================ */

		switch (rbstate)
		{
		case RBS_RUN:
		case RBS_JUMP:
			Pmuj_FSM();
			Wheel_Leg_Control();
			break;

		case RBS_STOP:
		default:
			Chassis_Zero();
			break;
		}

		prev_rbstate = rbstate;
		prev_rbflag = rbflag;

		// LQR_Control();
		// Yaw_Control();
		// Roll_Control();
		// LegLength_Control();
		// VMC_IK();

		/* ================ 电机指令 ================ */

		Chassis_Motor_Transmit();

		osDelay(1);
	}
}

/************************
 * @brief 初始化电机控制
 *
 ************************/
void Motor_Enable(void)
{
	for (int a = 1; a <= 4; a++)
	{
		AK_Motor_MIT_Enable(BSP_PORT1, a);
		osDelay(1);
	}
}

void Motor_Disable(void)
{
	for (int a = 1; a <= 4; a++)
	{
		AK_Motor_MIT_Disable(BSP_PORT1, a);
		osDelay(1);
	}
}

void Chassis_Zero(void)
{
	set.hip_torque[LF] = 0;
	set.hip_torque[LB] = 0;
	set.hip_torque[RF] = 0;
	set.hip_torque[RB] = 0;
	set.hub_torque[LEFT] = 0;
	set.hub_torque[RIGHT] = 0;
}

// debug variable
float ttphi1 = 0, ttphi4 = 0, tttp = 0, ttf0 = 0;

float dleg[2][2] = {0};

// 腿正解
void Wheel_Leg_Attitude_Calc(void)
{
	// left 反转
	VMC_5bar_FK(
		&leg[LEFT], PI / 2.0f + ak10[LF].angle, PI / 2.0f + ak10[LB].angle, ins.Pitch_Angle, ins.Pitch_Gyro, 0.001f);
	VMC_5bar_FK(
		&leg[RIGHT], PI / 2.0f - ak10[RF].angle, PI / 2.0f - ak10[RB].angle, ins.Pitch_Angle, ins.Pitch_Gyro, 0.001f);

	Leg_Spd(ak10[LF].speed, ak10[LB].speed, PI / 2.0f + ak10[LF].angle, PI / 2.0f + ak10[LB].angle, dleg[LEFT]);
	Leg_Spd(-ak10[RF].speed, -ak10[RB].speed, PI / 2.0f - ak10[RF].angle, PI / 2.0f - ak10[RB].angle, dleg[RIGHT]);

	// leg[LEFT].d_L0 = dl0_flr[LEFT].update(leg[LEFT].L0);
	// leg[RIGHT].d_L0 = dl0_flr[RIGHT].update(leg[RIGHT].L0);

	OffGround_Detection(&leg[LEFT], ins.Accel[1]);
	OffGround_Detection(&leg[RIGHT], ins.Accel[1]);

	if (jump_state != JPS_NONE && jump_state != JPS_INIT)
	{
		rbflag.offground = true;
	}
	else
	{
		// rbflag.offground = leg[LEFT].is_offground && leg[RIGHT].is_offground;
		rbflag.offground = false;
	}
}

int ch_task_cnt = 0;
float body_width = BODY_WIDTH;

bool hipMotorSendEnable[4] = {true, true, true, true};

void Chassis_Motor_Transmit(void)
{
	ch_task_cnt = (ch_task_cnt + 1) % 4;

#ifndef ZERO_FORCE

	/// @brief 用 3508
	RM_Motor_Control_Transmit(
		BSP_PORT3,
		M3508_TX_ID_1,
		(RM_Motor_Control_t){
			HEXROLL_TORQUE_TO_CURRENT(set.hub_torque[LEFT]), HEXROLL_TORQUE_TO_CURRENT(set.hub_torque[RIGHT]), 0, 0});

	if (ch_task_cnt == 0 && hipMotorSendEnable[0] == true)
	{
		AK_Motor_MIT_Transmit(BSP_PORT1, HIP_LF_ID, 0, 0, 0, 0, set.hip_torque[LF]);
	}
	else if (ch_task_cnt == 1 && hipMotorSendEnable[1] == true)
	{
		AK_Motor_MIT_Transmit(BSP_PORT1, HIP_LB_ID, 0, 0, 0, 0, set.hip_torque[LB]);
	}
	else if (ch_task_cnt == 2 && hipMotorSendEnable[2] == true)
	{
		AK_Motor_MIT_Transmit(BSP_PORT1, HIP_RF_ID, 0, 0, 0, 0, set.hip_torque[RF]);
	}
	else if (ch_task_cnt == 3 && hipMotorSendEnable[3] == true)
	{
		AK_Motor_MIT_Transmit(BSP_PORT1, HIP_RB_ID, 0, 0, 0, 0, set.hip_torque[RB]);
	}

#else

	RM_Motor_Control_Transmit(BSP_PORT1, M3508_TX_ID_1, (RM_Motor_Control_t){0, 0, 0, 0});

	AK_Motor_MIT_Transmit(BSP_PORT2, HIP_LF_ID, 0, 0, 0, 0, 0);
	AK_Motor_MIT_Transmit(BSP_PORT2, HIP_LB_ID, 0, 0, 0, 0, 0);
	osDelay(1);
	AK_Motor_MIT_Transmit(BSP_PORT2, HIP_RF_ID, 0, 0, 0, 0, 0);
	AK_Motor_MIT_Transmit(BSP_PORT2, HIP_RB_ID, 0, 0, 0, 0, 0);

#endif
}

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void Wheel_Leg_Control(void)
{
	xl[0] = 0 - leg[LEFT].theta;
	xl[1] = 0 - dtheta_filter[LEFT].update(leg[LEFT].d_theta);
	xl[2] = set.x - set.x;
	xl[3] = set.v - ob.v;
	xl[4] = 0 - ins.Pitch_Angle;
	xl[5] = 0 - ins.Pitch_Gyro;

	LQR_Control(xl, ul, ul_temp, leg[LEFT].L0);

	xr[0] = 0 - leg[RIGHT].theta;
	xr[1] = 0 - dtheta_filter[RIGHT].update(leg[RIGHT].d_theta);
	xr[2] = set.x - ob.x;
	xr[3] = set.v - ob.v;
	xr[4] = 0 - ins.Pitch_Angle;
	xr[5] = 0 - ins.Pitch_Gyro;

	LQR_Control(xr, ur, ur_temp, leg[RIGHT].L0);

	/* ================================ 轮 解算 ================================ */

	turn_t = yaw_pid.UpdateEZ((set.yaw - ins.Yaw_TolAngle), 0,
							  -ins.Yaw_Gyro); // 这样计算更稳一点

	set.hub_torque[LEFT] = -ul[U_T] + turn_t; // left 反转
	set.hub_torque[RIGHT] = ur[U_T] + turn_t;

	if (rbflag.offground)
	{
		set.yaw = ins.Yaw_TolAngle;
		set.hub_torque[LEFT] = 0;
		set.hub_torque[RIGHT] = 0;
	}

	/* ================================ 腿 解算 ================================ */

	// set.roll = set.yaw * 0.2;
	// roll_t = -roll_pid.Update(set.roll, ins.Roll_Angle);
	// roll_t = 0;

	// roll 前馈
	Roll_Height_Calc(leg[LEFT].L0,
					 leg[RIGHT].L0,
					 body_width,
					 ins.Roll_Angle,
					 set.height,
					 &set.length[LEFT],
					 &set.length[RIGHT],
					 0.1,
					 0.4);
	// set.length[LEFT] = set.length[RIGHT] = set.height;

	/// @brief 腿推力 PID

	leg[LEFT].F0 = leg_ff * COSF(leg[LEFT].theta) // 前馈
				   + roll_t						  // roll 补偿
				   + length_pid[LEFT].UpdateEZ(set.length[LEFT] - leg[LEFT].L0, 0, dleg[LEFT][0]);

	leg[RIGHT].F0 = leg_ff * COSF(leg[RIGHT].theta) // 前馈
					- roll_t						// roll 补偿
					+ length_pid[RIGHT].UpdateEZ(set.length[RIGHT] - leg[RIGHT].L0, 0, dleg[RIGHT][0]);

	tp_alpha = tp_pid.Update(0, leg[LEFT].alpha - leg[RIGHT].alpha);

	leg[LEFT].Tp = ul[U_TP] + tp_alpha;
	leg[RIGHT].Tp = ur[U_TP] - tp_alpha;

	// leg[RIGHT].Tp = tttp;
	// leg[RIGHT].F0 = ttf0;

	/// @brief 正 VMC
	/// @bug tp 不知道为什么是负的
	VMC_5bar_IK(&leg[LEFT], -leg[LEFT].Tp, leg[LEFT].F0);
	VMC_5bar_IK(&leg[RIGHT], -leg[RIGHT].Tp, leg[RIGHT].F0);

	/// @brief 发送 buf
	set.hip_torque[LF] = leg[LEFT].torque_set[FRONT]; // left 反转
	set.hip_torque[LB] = leg[LEFT].torque_set[BACK];
	set.hip_torque[RF] = -leg[RIGHT].torque_set[FRONT];
	set.hip_torque[RB] = -leg[RIGHT].torque_set[BACK];

/* ================================ 发送 ================================ */

/// @brief 限幅
#define HIP_TORQUE_MAX 18
#define HUB_TORQUE_MAX 3
	Clampfp(&set.hip_torque[LF], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clampfp(&set.hip_torque[LB], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clampfp(&set.hip_torque[RF], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clampfp(&set.hip_torque[RB], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);

	Clampfp(&set.hub_torque[LEFT], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	Clampfp(&set.hub_torque[RIGHT], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
}

uint32_t jump_time = 0;
// 超时处理
uint32_t init_time = 2000;
uint32_t stretch_time = 120;
uint32_t stretch_damping_time = 10;
uint32_t shrink_time = 80;
uint32_t shrink_damping_time = 10;
uint32_t land_time = 120;

Ramp_t jump_f_ramp;
float leg_len = 0;
float d_leg_len = 0;

void Jump_FSM(void)
{
	switch (jump_state)
	{
		// 空闲
	case JPS_NONE:
	{
		if (rbstate == RBS_JUMP)
		{
			jump_state = JPS_INIT;
			jump_time = 0;
		}
	}
	break;

		// 蹲下准备
	case JPS_INIT:
	{

		if (jump_time >= init_time)
		{
		}
	}
	break;

		// 伸腿
	case JPS_STRETCH:
	{

		if (leg_len >= set.height || jump_time >= stretch_time)
		{
		}
	}
	break;

	// 伸腿缓冲
	case JPS_STRETCH_DAMPING:
	{

		if (jump_time >= stretch_damping_time)
		{
		}
	}
	break;

		// 缩腿
	case JPS_SHRINK:
	{

		if (leg_len <= set.height || jump_time >= shrink_time)
		{
		}
	}
	break;

	// 缓冲
	case JPS_SHRINK_DAMPING:
	{

		if (jump_time >= shrink_damping_time)
		{
		}
	}
	break;

		// 结束
	case JPS_LAND:
	{

		if (rbflag.offground == false || jump_time >= land_time)
		{
		}
	}
	break;
	}

	jump_time += 1;
}

enum class Pmuj
{
	none,
	wait,
	shrink,
	back,
	end,
};

Pmuj pmuj = Pmuj::none;

float avg_length = 0;
uint32_t pmuj_time = 0;

void Pmuj_FSM(void)
{
	if (rbstate == RBS_JUMP && pmuj_time < 300)
	{
		pmuj_time++;
		avg_length = (leg[LEFT].L0 + leg[RIGHT].L0) * 0.5;

		leg_ff = -10;
		set.height = 0.1;
	}
	else
	{
		pmuj_time = 0;
		leg_ff = ROD2_MASS * GRAVITY_ACCEL / 2;
	}
}

// jump()
// {
// 	/*跳跃模式的常态控制，首先计算平衡力矩，在之后根据跳跃阶段，对平衡力矩的计算结果进行修改*/
// 	/*LQR平衡控制*/
// 	tor_balance = TorBalance_Calc(&ChassisNowState, LQR_gain, tmp_mask);
// 	/*计算同步腿摆角的力矩-防扭叉控制*/
// 	float err_synch = tool::annular_err_min(LegPos_L.agl, LegPos_R.agl,
// __TOOL_2PI); 	tool::deadzone_float(&err_synch, 0.01); /*死区*/ /*左腿 - 右腿*/
// 	float d_err_synch = LegPos_L.d_agl - LegPos_R.d_agl;
// 	float tor_synch_l = 50 * (0 - err_synch) + 5 * (0 - d_err_synch);
// 	float tor_synch_r = 50 * (err_synch - 0) + 5 * (d_err_synch - 0);
// 	/*yaw轴转向控制*/
// 	float yaw_err = tool::annular_err_min(ChassisAimState.yaw,
// ChassisNowState.yaw, __TOOL_2PI); 	float tor_yaw = 5.0 * (yaw_err) + 0.5 * (0
// - ChassisNowState.d_yaw); 	tool::limit_float(&tor_yaw, -2.0, 2.0);
// 	/*腿长控制，提前声明变量*/
// 	float tor_len_l;
// 	float tor_len_r;
// 	switch (GetChassisState())
// 	{
// 	/*缩短腿长，当腿长缩到较短时，进入到下一状态*/
// 	case cha_jump_initing:
// 	{
// 		/*缩短腿长*/
// 		tool::trapezoidal(LegPos_L.len, 0.13, 0.001, &AimPos_L.len);
// 		tool::trapezoidal(LegPos_R.len, 0.13, 0.001, &AimPos_R.len);
// 		/*计算维持腿长的力矩-腿长控制*/
// 		tor_len_l = 800 * ((AimPos_L.len) - LegPos_L.len) - 80 * LegPos_L.d_len;
// 		tor_len_r = 800 * ((AimPos_R.len) - LegPos_R.len) - 80 * LegPos_R.d_len;
// 		/*进入伸腿阶段*/
// 		if (AimPos_L.len <= 0.14 && AimPos_R.len <= 0.14)
// 		{
// 			SetChassisState(cha_jump_stretch);
// 		}
// 	}
// 	break;
// 	/*伸长腿长到最大，给vmc的力矩梯形增大，当腿长较长时，进入到下一状态*/
// 	case cha_jump_stretch:
// 	{
// 		/*跳跃的力矩*/
// 		tool::trapezoidal(VMCTor_Aim_L.force, 500, 2, &tor_len_l);
// 		tool::trapezoidal(VMCTor_Aim_R.force, 500, 2, &tor_len_r);
// 		/*退出条件：腿长达到最大*/
// 		if (LegPos_L.len >= 0.36 && LegPos_L.len >= 0.36)
// 		{
// 			SetChassisState(cha_jump_shrink);
// 		}
// 		/*超时退出状态机*/
// 		static uint16_t jump_stretch_time_count = 0;
// 		jump_stretch_time_count += __CHASSIS_TASK_DELAY;
// 		if (jump_stretch_time_count >= 2000)
// 		{
// 			jump_stretch_time_count = 0;
// 			SetChassisModeAndState(CHA_INIT_MODE, cha_init_initing);
// 		}
// 	}
// 	break;
// 	case cha_jump_shrink:
// 	{ /*收腿*/
// 		tor_balance = {0};
// 		/*缩短腿长*/
// 		tool::trapezoidal(LegPos_L.len, 0.16, 0.0008, &AimPos_L.len);
// 		tool::trapezoidal(LegPos_R.len, 0.16, 0.0008, &AimPos_R.len);
// 		tool::limit_float(&AimPos_L.len, __LEG_LEN_MIN, __LEG_LEN_MAX);
// /*限幅目标腿长*/ 		tool::limit_float(&AimPos_R.len, __LEG_LEN_MIN,
// __LEG_LEN_MAX); /*限幅目标腿长*/
// 		/*计算维持腿长的力矩-腿长控制*/
// 		tor_len_l = 50 * ((AimPos_L.len) - LegPos_L.len) - 80 * LegPos_L.d_len;
// 		tor_len_r = 50 * ((AimPos_R.len) - LegPos_R.len) - 80 * LegPos_R.d_len;
// 		/*使腿摆动到垂直地面,借用tor_balance传入这个力矩*/
// 		float tor_fly_l = 200 * tool::annular_err_min(0,
// ChassisNowState.leg[LEFT].theta, __TOOL_2PI) + 20 * (0 - LegPos_L.d_agl);
// 		float tor_fly_r = 200 * tool::annular_err_min(0,
// ChassisNowState.leg[RIGHT].theta, __TOOL_2PI) + 20 * (0 - LegPos_R.d_agl);
// 		tor_balance.tor_l = tor_fly_l;
// 		tor_balance.tor_r = tor_fly_r;
// 		/*进入飞行阶段*/
// 		if (AimPos_L.len <= 0.20 && AimPos_R.len <= 0.20)
// 		{
// 			SetChassisState(cha_jump_ending);
// 		}
// 		/*超时退出状态机*/
// 		static uint16_t jump_shrink_time_count = 0;
// 		jump_shrink_time_count += __CHASSIS_TASK_DELAY;
// 		if (jump_shrink_time_count >= 1000)
// 		{
// 			jump_shrink_time_count = 0;
// 			SetChassisModeAndState(CHA_INIT_MODE, cha_init_initing);
// 		}
// 	}
// 	break;
// 	case cha_jump_ending:
// 	{
// 		tor_balance = {0};
// 		/*使腿摆动到垂直地面,借用tor_balance传入这个力矩*/
// 		float tor_fly_l = 200 * tool::annular_err_min(0,
// ChassisNowState.leg[LEFT].theta, __TOOL_2PI) + 10 * (0 - LegPos_L.d_agl);
// 		float tor_fly_r = 200 * tool::annular_err_min(0,
// ChassisNowState.leg[RIGHT].theta, __TOOL_2PI) + 10 * (0 - LegPos_R.d_agl);
// 		tor_balance.tor_l = tor_fly_l;
// 		tor_balance.tor_r = tor_fly_r;
// 		/*滞空一段时间后伸长腿来缓冲*/
// 		static uint16_t jump_ending_time_count = 0;
// 		jump_ending_time_count += __CHASSIS_TASK_DELAY;
// 		if (jump_ending_time_count >= 200) /*滞空时间200ms*/
// 		{
// 			tool::trapezoidal(LegPos_L.len, 0.250, 0.001, &AimPos_L.len);
// 			tool::trapezoidal(LegPos_R.len, 0.250, 0.001, &AimPos_R.len);
// 			tor_len_l = 500 * ((AimPos_L.len) - LegPos_L.len) - 100 *
// LegPos_L.d_len; 			tor_len_r = 500 * ((AimPos_R.len) - LegPos_R.len) - 100 *
// LegPos_R.d_len; 			if (AimPos_L.len >= 0.24 && AimPos_R.len >= 0.24)
// 			{
// 				jump_ending_time_count = 0;
// 				SetChassisModeAndState(CHA_MOVE_MODE, cha_move_initing);
// 			}
// 			if (jump_ending_time_count >= 1000)
// 			{
// 				jump_ending_time_count = 0;
// 				SetChassisModeAndState(CHA_INIT_MODE, cha_init_initing);
// 			}
// 		}
// 	}
// 	break;
// 	default:
// 	{
// 		tor_len_l = 800 * ((AimPos_L.len) - LegPos_L.len) - 80 * LegPos_L.d_len
// + 30; 		tor_len_r = 800 * ((AimPos_R.len) - LegPos_R.len) - 80 * LegPos_R.d_len
// + 30;
// 	};
// 	}
// 	/*信息传递到下层*/
// 	SetAimTorVMC(&VMCTor_Aim_L, tor_len_l + tor_balance.force_l, tor_synch_l +
// tor_balance.tor_l, tor_balance.tor_wheel_l + tor_yaw);
// 	SetAimTorVMC(&VMCTor_Aim_R, tor_len_r + tor_balance.force_r, tor_synch_r +
// tor_balance.tor_r, tor_balance.tor_wheel_r - tor_yaw);
// }

/// @brief yaw 控制
void Yaw_Control(void)
{
	// Ramp_Update(&yaw_ramp, att.totalyaw, set.yaw, (.25f / 100.f));
	// PID_Update(&yaw_pid, att.totalyaw, set.yaw);
}

void LQR_Control_(void)
{
}

float phi_diff = 0,	  // 两腿角度差
	phi_slope = 0,	  // 地面倾斜角
	delta_length = 0, // 腿长差
	slope_ratio = 1.2;

/// @brief 腿长计算
/// @param ll 左腿长 当前
/// @param lr 右腿长 当前
/// @param w 机体宽度
/// @param r 机体 ins.roll
/// @param h 机体中心里地高度 目标
/// @param ltl[out] 左腿长 目标
/// @param ltr[out] 右腿长 目标
/// @param min_len 最小腿长
/// @param max_len 最大腿长
/// @note 按照右手直角坐标系，xyz前左上，绕xzy轴逆时针为roll pitch yaw
void Roll_Height_Calc(
	float ll, float lr, float w, float r, float h, float *ltl, float *ltr, float min_len, float max_len)
{
	phi_diff = std::atanf((ll - lr) / w);
	phi_slope = phi_diff + r;
	delta_length = w / 2 * std::tanf(phi_slope * slope_ratio);

	*ltl = h + delta_length;
	*ltr = h - delta_length;
}
