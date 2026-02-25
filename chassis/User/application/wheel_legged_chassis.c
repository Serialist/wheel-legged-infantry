/***********************************************
 * @file wheel_legged_chassis.c
 * @author Serialist (ba3pt@chd.edu.cn)
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

#include "wheel_legged_chassis.h"
#include "observer.h"
#include "lqr.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "INS_task.h"
#include "bsp_dwt.h"
#include "dt7.h"
#include "user_lib.h"
#include "vmc-dm.h"
#include "debug.h"
#include "filter.h"
#include "motor.h"

/* ================================================================ micro ================================================================ */

/* ================================================================ variable ================================================================ */

extern Motor_AK_RxData_t ak10[4];

Robo_Status_t robo_status;
Robo_Attitude_t att;
JUMP_State_t jump_state = JPS_NONE;

PID_Typedef pid_tpl = {0},
			pid_tpr = {0},
			leglength_pid_l = {0},
			leglength_pid_r = {0},
			yaw_pid = {0},
			roll_pid = {0},
			tp_pid = {0},
			tp_offground_pid = {0};

Wheel_Leg_Target_t set;
VMC_t leg_l, leg_r;
Robo_Flag_t rbflag;

float turn_t;	// yaw轴补偿
float tp_alpha; // 劈叉
float tplqrl;
float tlqrl;
float tplqrr;
float tlqrr;
float leg_force_l = 0;
float leg_force_r = 0;
float leg_force = 0;

// 状态变量和控制量
float xl[6], ul[2];
float xr[6], ur[2];

// debug variable
float t1 = 0, t2 = 0;

/* ================================================================ prototype ================================================================ */

void Chassis_PID_Init(void);
void Chassis_Motor_Transmit(void);
void Wheel_Leg_Control(void);
void Wheel_Leg_Attitude_Calc(void);
void Control_Get(void);
void Motor_Enable(void);
void Jump_FSM(void);

/* ================================================================ function ================================================================ */

void Chassis_Task(void const *argument)
{
	Chassis_PID_Init();
	VMC_Init(&leg_l);
	VMC_Init(&leg_r);
	Motor_Enable();
	set.left_length = set.right_length = 0.15f;

	for (;;)
	{
		/* ================ 状态更新 ================ */

		Wheel_Leg_Attitude_Calc();

		/* ================ 控制 ================ */

		Control_Get();
		Jump_FSM();
		Wheel_Leg_Control();

		/* ================ 电机指令 ================ */

		Chassis_Motor_Transmit();
	}
}

void Chassis_PID_Init(void)
{
	PID_init(&leglength_pid_l, 400, 0, 9000, 120, 0); // 腿长 left
	PID_init(&leglength_pid_r, 400, 0, 9000, 120, 0); // 腿长 right
	PID_init(&yaw_pid, 0.12f, 0, 0.8f, 0, 0);		  // yaw
	PID_init(&roll_pid, 0.8f, 0, 0, 30.0f, 0);		  // roll
	PID_init(&tp_pid, 10, 0, 2, 3, 0);				  // 劈叉

	// 腿摆角扭矩pid，用于板凳模型
	PID_init(&pid_tpl, 80, 0, 400, 10, 0);
	PID_init(&pid_tpr, 80, 0, 400, 10, 0);

	PID_init(&tp_offground_pid, 80, 0, 400, 0, 0);
}

/************************
 * @brief 初始化电机控制
 *
 ************************/
void Motor_Enable(void)
{
	for (int a = 1; a <= 4; a++)
	{
		AK_Motor_MIT_Enable(a);
		osDelay(1);
	}
}

// debug variable
float ttphi1 = 0, ttphi4 = 0, tttp = 0, ttf0 = 0;

void Wheel_Leg_Attitude_Calc(void)
{
	VMC_5bar_FK(&leg_l,
				PI / 2.0f + ak10[LF].angle, // left 反转
				PI / 2.0f + ak10[LB].angle,
				att.pitch, att.vpitch,
				0.003f);
	VMC_5bar_FK(&leg_r,
				PI / 2.0f - ak10[RF].angle,
				PI / 2.0f - ak10[RB].angle,
				att.pitch, att.vpitch,
				0.003f);

	OffGround_Detection(&leg_l, att.az);
	OffGround_Detection(&leg_r, att.az);

	if (jump_state != JPS_NONE && jump_state != JPS_INIT)
	{
		rbflag.above = true;
	}
	else
	{
		// rbflag.above = leg_l.is_offground && leg_r.is_offground;
		rbflag.above = false;
	}
}

void Chassis_Motor_Transmit(void)
{
	if (rc_ctrl.rc.s[S_L] == UP)
	{
		for (int i = 0; i < 6; i++)
			set.torque[i] = 0;
	}

	/// @brief 用 3508
	RM_Motor_Transmit(&hcan1, M3508_TX_ID_2,
					  0,
					  HEXROLL_TORQUE_TO_CURRENT(set.torque[WR]),
					  HEXROLL_TORQUE_TO_CURRENT(set.torque[WL]),
					  0);
	osDelay(1);

	// MIT模式下发送
	AK_Motor_MIT_Transmit(HIP_LF_ID, 0, 0, 0, 0, set.torque[LF]);
	AK_Motor_MIT_Transmit(HIP_LB_ID, 0, 0, 0, 0, set.torque[LB]);
	osDelay(1);
	AK_Motor_MIT_Transmit(HIP_RF_ID, 0, 0, 0, 0, set.torque[RF]);
	AK_Motor_MIT_Transmit(HIP_RB_ID, 0, 0, 0, 0, set.torque[RB]);
	osDelay(1);

	// RM_Motor_Transmit(&hcan1, M3508_TX_ID_2,
	// 				  0,
	// 				  HEXROLL_TORQUE_TO_CURRENT(0),
	// 				  HEXROLL_TORQUE_TO_CURRENT(0),
	// 				  0);
	// osDelay(1);

	// AK_Motor_MIT_Transmit(HIP_LF_ID, 0, 0, 0, 0, 0);
	// AK_Motor_MIT_Transmit(HIP_LB_ID, 0, 0, 0, 0, 0);
	// osDelay(1);
	// AK_Motor_MIT_Transmit(HIP_RF_ID, 0, 0, 0, 0, 0);
	// AK_Motor_MIT_Transmit(HIP_RB_ID, 0, 0, 0, 0, 0);
	// osDelay(1);

	// 1. 使用返回状态 HAL_BUSY
	// if(HAL_CAN_AddTxMessage()==HAL_BUSY)
	// osDelay(1);
	// 2. 查看邮箱状态
	// while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
	// 3. interrupt
	// HAL_CAN_AddTxMessage
}

uint8_t last_switch = 0;

void Control_Get(void)
{
	set.v = rc_ctrl.rc.ch[L_Y] * 3.5f / 660.0f;
	set.yaw -= rc_ctrl.rc.ch[L_X] * 0.001f;
	set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
	set.roll = -rc_ctrl.rc.ch[R_X] * 45.0f / 660.0f;
	leg_force = rc_ctrl.rc.ch[L_Z] * 50 / 660.0f;

	if (set.v != 0)
		set.x = ob.x;

	if (rc_ctrl.rc.s[S_L] == MID || last_switch == DOWN) // 正常行驶
	{
		robo_status = RBS_RUN;

		leglength_pid_l.Kp = 400;
		leglength_pid_l.Kd = 9000;
		leglength_pid_l.max_out = 120;

		leglength_pid_r.Kp = 400;
		leglength_pid_r.Kd = 9000;
		leglength_pid_r.max_out = 120;
	}
	else if (rc_ctrl.rc.s[S_L] == DOWN && jump_state == JPS_NONE)
	{
		robo_status = RBS_JUMP;

		leglength_pid_l.Kp = 800;
		leglength_pid_l.Kd = 15000;
		leglength_pid_l.max_out = 300;

		leglength_pid_r.Kp = 800;
		leglength_pid_r.Kd = 15000;
		leglength_pid_r.max_out = 300;
	}
	else
	{
		set.v = 0;
		set.yaw = att.totalyaw;
		set.roll = 0;
		set.left_length = set.right_length = 0.2f;
		set.x = ob.x;
	}

	last_switch = rc_ctrl.rc.s[S_L];
}

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void Wheel_Leg_Control(void)
{
	xl[0] = leg_l.theta;
	xl[1] = leg_l.d_theta;
	xl[2] = ob.x - set.x;
	xl[3] = ob.v - set.v;
	xl[4] = att.pitch;
	xl[5] = att.vpitch;

	xr[0] = leg_r.theta;
	xr[1] = leg_r.d_theta;
	xr[2] = ob.x - set.x;
	xr[3] = ob.v - set.v;
	xr[4] = att.pitch;
	xr[5] = att.vpitch;

	LQR_Control(xl, ul, leg_l.L0);
	LQR_Control(xr, ur, leg_r.L0);

	// 应为 u = -kx，所以这里取负
	tlqrl = -ul[0];
	tplqrl = -ul[1];
	tlqrr = -ur[0];
	tplqrr = -ur[1];

	// if (rbflag.above)
	// {
	// 	for (int i = 0; i < 6; i++)
	// 	{
	// 		lqr_k_l[0][i] = 0.0f;
	// 		lqr_k_r[0][i] = 0.0f;
	// 		if (i != 0 && i != 1)
	// 		{
	// 			lqr_k_l[1][i] = 0.0f;
	// 			lqr_k_r[1][i] = 0.0f;
	// 		}
	// 	}

	// 	// tlqrl = tlqrr = 0;
	// 	// tplqrl = tp_offground_pid.Kp * leg_l.theta + tp_offground_pid.Kd * leg_l.d_theta;
	// 	// tplqrr = tp_offground_pid.Kp * leg_r.theta + tp_offground_pid.Kd * leg_r.d_theta;
	// }

	// tplqrl = PID_Update(&pid_tpl, 0, leg_l.alpha);
	// tplqrr = PID_Update(&pid_tpr, 0, leg_r.alpha);

	/* ================================ 轮 解算 ================================ */

	turn_t = yaw_pid.Kp * (set.yaw - att.totalyaw) - yaw_pid.Kd * att.vyaw; // 这样计算更稳一点
	if (rbflag.above)
		turn_t = 0;
	set.torque[WL] = -(tlqrl - turn_t); // left 反转
	set.torque[WR] = tlqrr + turn_t;

	/* ================================ 腿 解算 ================================ */

	// set.left_length = set.height / arm_cos_f32(leg_l.theta);
	// set.right_length = set.height / arm_cos_f32(leg_r.theta);

	/// @brief 腿推力 PID
	leg_l.F0 = 55.0f * arm_cos_f32(leg_l.theta) +
			   PID_Update(&leglength_pid_l, set.left_length, leg_l.L0) +
			   leg_force +
			   leg_force_l;
	/// @bug 这里不应该加负号，我怀疑是上面正运动学角度反了
	leg_r.F0 = -55.0f * arm_cos_f32(leg_r.theta) +
			   -PID_Update(&leglength_pid_r, set.right_length, leg_r.L0) +
			   -leg_force +
			   -leg_force_r;

	tp_alpha = PID_Update(&tp_pid, 0, leg_l.alpha - leg_r.alpha);

	leg_l.Tp = tplqrl + tp_alpha;
	leg_r.Tp = tplqrr - tp_alpha;

	// leg_r.Tp = tttp;
	// leg_r.F0 = ttf0;

	/// @brief 正 VMC
	VMC_5bar_IK(&leg_l,
				leg_l.Tp,
				leg_l.F0);
	VMC_5bar_IK(&leg_r,
				leg_r.Tp,
				leg_r.F0);

	/// @brief 发送 buf
	set.torque[LF] = -leg_l.torque_set[BACK]; // left 反转
	set.torque[LB] = -leg_l.torque_set[FRONT];
	set.torque[RF] = leg_r.torque_set[FRONT];
	set.torque[RB] = leg_r.torque_set[BACK];

/* ================================ 发送 ================================ */

/// @brief 限幅
#define HIP_TORQUE_MAX 35.0f
#define HUB_TORQUE_MAX 2.5f
	Clamp(&set.torque[0], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[1], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[2], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[3], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);

	Clamp(&set.torque[4], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	Clamp(&set.torque[5], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);

	// set.torque[0] = ClampAbsf(set.torque[0], HIP_TORQUE_MAX);
	// set.torque[1] = ClampAbsf(set.torque[1], HIP_TORQUE_MAX);
	// set.torque[2] = ClampAbsf(set.torque[2], HIP_TORQUE_MAX);
	// set.torque[3] = ClampAbsf(set.torque[3], HIP_TORQUE_MAX);

	// set.torque[4] = ClampAbsf(set.torque[4], HUB_TORQUE_MAX);
	// set.torque[5] = ClampAbsf(set.torque[5], HUB_TORQUE_MAX);
}

uint32_t jump_time = 0;
// ms
uint32_t stretch_time = 100;
uint32_t shrink_time = 250;
uint32_t air_time = 200;
uint32_t end_time = 100;
// N
float stretch_force = 300;
float shrink_force = -200;
float air_force = 50;

void Jump_FSM(void)
{
	switch (jump_state)
	{
	case JPS_NONE:
	{
		leg_force_l = leg_force_r = 0;
		set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
		if (robo_status == RBS_JUMP)
		{
			jump_state = JPS_INIT;
			jump_time = 0;
		}
	}
	break;

	case JPS_INIT:
	{
		set.left_length = set.right_length = .15f;
		if (jump_time >= 500)
		{
			jump_state = JPS_STRETCH;
			jump_time = 0;
		}
	}
	break;

	case JPS_STRETCH:
	{
		set.left_length = set.right_length = .4f;
		leg_force_l = leg_force_r = stretch_force;
		if (((leg_l.L0 + leg_r.L0) / 2) >= .35f || jump_time >= stretch_time)
		{
			jump_state = JPS_SHRINK;
			jump_time = 0;
		}
	}
	break;

	case JPS_SHRINK:
	{
		set.left_length = set.right_length = .15f;
		leg_force_l = leg_force_r = shrink_force;
		if (((leg_l.L0 + leg_r.L0) / 2) <= .18f || jump_time >= shrink_time)
		{
			jump_state = JPS_AIR;
			jump_time = 0;
		}
	}
	break;

	case JPS_AIR:
	{

		set.left_length = set.right_length = .15f;
		leg_force_l = leg_force_r = air_force;
		if (/* (leg_l.is_offground == false && leg_r.is_offground == false) || */ /* (leg_l.d_L0 + leg_r.d_L0) / 2 */ jump_time >= air_time)
		{
			jump_state = JPS_END;
			jump_time = 0;
		}
	}
	break;

	case JPS_END:
	{
		if ((leg_l.is_offground == false && leg_r.is_offground == false) || jump_time >= end_time)
		{
			jump_state = JPS_NONE;
			jump_time = 0;
		}
	}
	break;
	}
	jump_time += 3;
}

// jump()
// {
// 	/*跳跃模式的常态控制，首先计算平衡力矩，在之后根据跳跃阶段，对平衡力矩的计算结果进行修改*/
// 	/*LQR平衡控制*/
// 	tor_balance = TorBalance_Calc(&ChassisNowState, LQR_gain, tmp_mask);
// 	/*计算同步腿摆角的力矩-防扭叉控制*/
// 	float err_synch = tool::annular_err_min(LegPos_L.agl, LegPos_R.agl, __TOOL_2PI);
// 	tool::deadzone_float(&err_synch, 0.01); /*死区*/ /*左腿 - 右腿*/
// 	float d_err_synch = LegPos_L.d_agl - LegPos_R.d_agl;
// 	float tor_synch_l = 50 * (0 - err_synch) + 5 * (0 - d_err_synch);
// 	float tor_synch_r = 50 * (err_synch - 0) + 5 * (d_err_synch - 0);
// 	/*yaw轴转向控制*/
// 	float yaw_err = tool::annular_err_min(ChassisAimState.yaw, ChassisNowState.yaw, __TOOL_2PI);
// 	float tor_yaw = 5.0 * (yaw_err) + 0.5 * (0 - ChassisNowState.d_yaw);
// 	tool::limit_float(&tor_yaw, -2.0, 2.0);
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
// 		tool::limit_float(&AimPos_L.len, __LEG_LEN_MIN, __LEG_LEN_MAX); /*限幅目标腿长*/
// 		tool::limit_float(&AimPos_R.len, __LEG_LEN_MIN, __LEG_LEN_MAX); /*限幅目标腿长*/
// 		/*计算维持腿长的力矩-腿长控制*/
// 		tor_len_l = 50 * ((AimPos_L.len) - LegPos_L.len) - 80 * LegPos_L.d_len;
// 		tor_len_r = 50 * ((AimPos_R.len) - LegPos_R.len) - 80 * LegPos_R.d_len;
// 		/*使腿摆动到垂直地面,借用tor_balance传入这个力矩*/
// 		float tor_fly_l = 200 * tool::annular_err_min(0, ChassisNowState.leg_l.theta, __TOOL_2PI) + 20 * (0 - LegPos_L.d_agl);
// 		float tor_fly_r = 200 * tool::annular_err_min(0, ChassisNowState.leg_r.theta, __TOOL_2PI) + 20 * (0 - LegPos_R.d_agl);
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
// 		float tor_fly_l = 200 * tool::annular_err_min(0, ChassisNowState.leg_l.theta, __TOOL_2PI) + 10 * (0 - LegPos_L.d_agl);
// 		float tor_fly_r = 200 * tool::annular_err_min(0, ChassisNowState.leg_r.theta, __TOOL_2PI) + 10 * (0 - LegPos_R.d_agl);
// 		tor_balance.tor_l = tor_fly_l;
// 		tor_balance.tor_r = tor_fly_r;
// 		/*滞空一段时间后伸长腿来缓冲*/
// 		static uint16_t jump_ending_time_count = 0;
// 		jump_ending_time_count += __CHASSIS_TASK_DELAY;
// 		if (jump_ending_time_count >= 200) /*滞空时间200ms*/
// 		{
// 			tool::trapezoidal(LegPos_L.len, 0.250, 0.001, &AimPos_L.len);
// 			tool::trapezoidal(LegPos_R.len, 0.250, 0.001, &AimPos_R.len);
// 			tor_len_l = 500 * ((AimPos_L.len) - LegPos_L.len) - 100 * LegPos_L.d_len;
// 			tor_len_r = 500 * ((AimPos_R.len) - LegPos_R.len) - 100 * LegPos_R.d_len;
// 			if (AimPos_L.len >= 0.24 && AimPos_R.len >= 0.24)
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
// 		tor_len_l = 800 * ((AimPos_L.len) - LegPos_L.len) - 80 * LegPos_L.d_len + 30;
// 		tor_len_r = 800 * ((AimPos_R.len) - LegPos_R.len) - 80 * LegPos_R.d_len + 30;
// 	};
// 	}
// 	/*信息传递到下层*/
// 	SetAimTorVMC(&VMCTor_Aim_L, tor_len_l + tor_balance.force_l, tor_synch_l + tor_balance.tor_l, tor_balance.tor_wheel_l + tor_yaw);
// 	SetAimTorVMC(&VMCTor_Aim_R, tor_len_r + tor_balance.force_r, tor_synch_r + tor_balance.tor_r, tor_balance.tor_wheel_r - tor_yaw);
// }
