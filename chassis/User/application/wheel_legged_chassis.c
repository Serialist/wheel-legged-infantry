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

Robo_Status_t robo_status;			// 机器人模式
Robo_Attitude_t att;				// 现在姿态
JUMP_State_t jump_state = JPS_NONE; // 跳跃状态机

PID_Typedef pid_tpl = {0},
			pid_tpr = {0},
			*leg_pid[2],
			length_pid[2] = {0},
			jump_length_pid[2] = {0},
			yaw_pid = {0},
			roll_pid = {0},
			tp_pid = {0},
			tp_offground_pid = {0};

Wheel_Leg_Target_t set; // 目标值
VMC_t leg[2];			// 腿解算
Robo_Flag_t rbflag;		// 机器人状态
Ramp_t ramp_leg_length, // 腿长斜坡函数
	v_ramp;				// 速度斜坡函数

float turn_t;	// yaw轴补偿
float f0_roll;	// roll轴补偿
float tp_alpha; // 劈叉

// lqr 输出
float tplqrl, tlqrl, tplqrr, tlqrr;

// 状态变量
float xl[6], ul[2];
// 控制量
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
	VMC_Init(&leg[LEFT]);
	VMC_Init(&leg[RIGHT]);
	Motor_Enable();
	set.length = 0.13f;
	Ramp_Init(&ramp_leg_length, .1f, -0.0005f, 0.0005f);
	Ramp_Init(&v_ramp, 0, -10.f, 10.f);

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
	PID_init(&length_pid[LEFT], 600, 0, 12000, 120, 0);		   // 腿长 left
	PID_init(&length_pid[RIGHT], 600, 0, 12000, 120, 0);	   // 腿长 right
	PID_init(&jump_length_pid[LEFT], 1000, 0, 25000, 0, 300);  // jump 腿长 left
	PID_init(&jump_length_pid[RIGHT], 1000, 0, 25000, 0, 300); // jump 腿长 right
	PID_init(&yaw_pid, 0.12f, 0, 0.8f, 0, 0);				   // yaw
	PID_init(&roll_pid, .8f, 0, .05f, .2f, 0);				   // roll
	PID_init(&tp_pid, 10, 0, 2, 3, 0);						   // 劈叉

	// 腿摆角扭矩pid，用于板凳模型
	PID_init(&pid_tpl, 14, 0, 3, 10, 0);
	PID_init(&pid_tpr, 14, 0, 3, 10, 0);

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
	VMC_5bar_FK(&leg[LEFT],
				PI / 2.0f + ak10[LF].angle, // left 反转
				PI / 2.0f + ak10[LB].angle,
				att.pitch, att.vpitch,
				0.003f);
	VMC_5bar_FK(&leg[RIGHT],
				PI / 2.0f - ak10[RF].angle,
				PI / 2.0f - ak10[RB].angle,
				att.pitch, att.vpitch,
				0.003f);

	OffGround_Detection(&leg[LEFT], att.az);
	OffGround_Detection(&leg[RIGHT], att.az);

	if (jump_state != JPS_NONE && jump_state != JPS_INIT)
	{
		rbflag.above = true;
	}
	else
	{
		rbflag.above = leg[LEFT].is_offground /* || leg[RIGHT].is_offground */;
	}
}

void Chassis_Motor_Transmit(void)
{
	if (rc_ctrl.rc.s[S_L] == UP)
	{
		set.hip_torque[LF] = 0;
		set.hip_torque[LB] = 0;
		set.hip_torque[RF] = 0;
		set.hip_torque[RB] = 0;
		set.hub_torque[LEFT] = 0;
		set.hub_torque[RIGHT] = 0;
	}

	/// @brief 用 3508
	RM_Motor_Transmit(&hcan1, M3508_TX_ID_2,
					  0,
					  HEXROLL_TORQUE_TO_CURRENT(set.hub_torque[RIGHT]),
					  HEXROLL_TORQUE_TO_CURRENT(set.hub_torque[LEFT]),
					  0);
	osDelay(1);

	// MIT模式下发送
	AK_Motor_MIT_Transmit(HIP_LF_ID, 0, 0, 0, 0, set.hip_torque[LF]);
	AK_Motor_MIT_Transmit(HIP_LB_ID, 0, 0, 0, 0, set.hip_torque[LB]);
	osDelay(1);
	AK_Motor_MIT_Transmit(HIP_RF_ID, 0, 0, 0, 0, set.hip_torque[RF]);
	AK_Motor_MIT_Transmit(HIP_RB_ID, 0, 0, 0, 0, set.hip_torque[RB]);
	osDelay(1);
}

uint8_t last_switch = 0;

float aaa;

void Control_Get(void)
{
	set.v = Ramp_Update(&v_ramp, rc_ctrl.rc.ch[L_Y] * 3.5f / 660.0f, 0.003f);
	set.yaw -= rc_ctrl.rc.ch[L_X] * 0.0015f;
	set.length = Ramp_Update(&ramp_leg_length,
							 Remapf((float)rc_ctrl.rc.ch[R_Y], -660.0f, 660.0f, 0.1f, 0.35f),
							 3);
	// set.roll = -rc_ctrl.rc.ch[R_X] * 30.0f / 660.0f;
	set.roll = 0;
	set.f0_force = rc_ctrl.rc.ch[L_Z] * 50 / 660.0f;

	if (set.v != 0)
		set.x = ob.x;

	if (rc_ctrl.rc.s[S_L] == MID || last_switch == DOWN) // 正常行驶
	{
		robo_status = RBS_RUN;

		leg_pid[LEFT] = &length_pid[LEFT];
		leg_pid[RIGHT] = &length_pid[RIGHT];
	}
	else if (rc_ctrl.rc.s[S_L] == DOWN && jump_state == JPS_NONE)
	{
		robo_status = RBS_JUMP;

		leg_pid[LEFT] = &jump_length_pid[LEFT];
		leg_pid[RIGHT] = &jump_length_pid[RIGHT];
	}
	else
	{
		set.v = 0;
		set.yaw = att.totalyaw;
		set.roll = 0;
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
	xl[0] = leg[LEFT].theta;
	xl[1] = leg[LEFT].d_theta;
	xl[2] = (ob.x - set.x - 2.f);
	xl[3] = (ob.v - set.v);
	xl[4] = att.pitch;
	xl[5] = att.vpitch;

	xr[0] = leg[RIGHT].theta;
	xr[1] = leg[RIGHT].d_theta;
	xr[2] = (ob.x - set.x - 1.5f);
	xr[3] = (ob.v - set.v);
	xr[4] = att.pitch;
	xr[5] = att.vpitch;

	LQR_Control(xl, ul, leg[LEFT].L0);
	LQR_Control(xr, ur, leg[RIGHT].L0);

	// 应为 u = -kx，所以这里取负
	tlqrl = -ul[0];
	tplqrl = -ul[1];
	tlqrr = -ur[0];
	tplqrr = -ur[1];

	if (rbflag.above)
	{
		tlqrl = 0;
		tlqrr = 0;
		set.x = ob.x;
		set.yaw = att.totalyaw;
		tplqrl = PID_Update(&pid_tpl, 0, leg[LEFT].theta);
		tplqrr = PID_Update(&pid_tpr, 0, leg[RIGHT].theta);
	}

	// 	// tlqrl = tlqrr = 0;
	// 	// tplqrl = tp_offground_pid.Kp * leg[LEFT].theta + tp_offground_pid.Kd * leg[LEFT].d_theta;
	// 	// tplqrr = tp_offground_pid.Kp * leg[RIGHT].theta + tp_offground_pid.Kd * leg[RIGHT].d_theta;
	// }

	/* ================================ 轮 解算 ================================ */

	turn_t = yaw_pid.Kp * (set.yaw - att.totalyaw) - yaw_pid.Kd * att.vyaw; // 这样计算更稳一点
	if (rbflag.above)
		turn_t = 0;
	set.hub_torque[LEFT] = -(tlqrl - turn_t); // left 反转
	set.hub_torque[RIGHT] = tlqrr + turn_t;

	/* ================================ 腿 解算 ================================ */

	// set.length = set.height / arm_cos_f32(leg[LEFT].theta);
	// set.right_length = set.height / arm_cos_f32(leg[RIGHT].theta);

	f0_roll = PID_Update(&roll_pid, set.roll, att.roll);

	/// @brief 腿推力 PID
	leg[LEFT].F0 = 55.0f * arm_cos_f32(leg[LEFT].theta) +
				   PID_Update(leg_pid[LEFT], set.length + f0_roll, leg[LEFT].L0) +
				   set.f0_force;
	/// @bug 这里不应该加负号，我怀疑是上面正运动学角度反了
	leg[RIGHT].F0 = -55.0f * arm_cos_f32(leg[RIGHT].theta) +
					-PID_Update(leg_pid[RIGHT], set.length - f0_roll, leg[RIGHT].L0) +
					-set.f0_force;

	tp_alpha = PID_Update(&tp_pid, 0, leg[LEFT].alpha - leg[RIGHT].alpha);

	leg[LEFT].Tp = tplqrl + tp_alpha;
	leg[RIGHT].Tp = tplqrr - tp_alpha;

	// leg[RIGHT].Tp = tttp;
	// leg[RIGHT].F0 = ttf0;

	/// @brief 正 VMC
	VMC_5bar_IK(&leg[LEFT],
				leg[LEFT].Tp,
				leg[LEFT].F0);
	VMC_5bar_IK(&leg[RIGHT],
				leg[RIGHT].Tp,
				leg[RIGHT].F0);

	/// @brief 发送 buf
	set.hip_torque[LF] = -leg[LEFT].torque_set[BACK]; // left 反转
	set.hip_torque[LB] = -leg[LEFT].torque_set[FRONT];
	set.hip_torque[RF] = leg[RIGHT].torque_set[FRONT];
	set.hip_torque[RB] = leg[RIGHT].torque_set[BACK];

/* ================================ 发送 ================================ */

/// @brief 限幅
#define HIP_TORQUE_MAX 35.0f
#define HUB_TORQUE_MAX 2.5f
	Clampfp(&set.hip_torque[LF], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clampfp(&set.hip_torque[LB], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clampfp(&set.hip_torque[RF], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clampfp(&set.hip_torque[RB], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);

	Clampfp(&set.hub_torque[LEFT], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	Clampfp(&set.hub_torque[RIGHT], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
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

Ramp_t jump_f_ramp;

void Jump_FSM(void)
{
	switch (jump_state)
	{
	case JPS_NONE:
	{
		set.f0_force = 0;
		if (robo_status == RBS_JUMP)
		{
			jump_state = JPS_INIT;
			jump_time = 0;
		}
	}
	break;

	case JPS_INIT:
	{
		set.length = .1f;
		if (jump_time >= 500)
		{
			Ramp_Init(&jump_f_ramp, leg[LEFT].L0, 0, (.25f / 200.f));

			jump_state = JPS_STRETCH;
			jump_time = 0;
		}
	}
	break;

	case JPS_STRETCH:
	{
		set.length = Ramp_Update(&jump_f_ramp, .35f, .003f); // 斜坡函数
		set.f0_force = stretch_force;
		if (((leg[LEFT].L0 + leg[RIGHT].L0) / 2) >= .35f || jump_time >= stretch_time)
		{
			Ramp_Init(&jump_f_ramp, leg[LEFT].L0, -(.25f / 200.f), 0);

			jump_state = JPS_SHRINK;
			jump_time = 0;
		}
	}
	break;

	case JPS_SHRINK:
	{
		set.length = Ramp_Update(&jump_f_ramp, .13f, .003f); // 斜坡函数
		set.f0_force = shrink_force;
		if (((leg[LEFT].L0 + leg[RIGHT].L0) / 2) <= .15f || jump_time >= shrink_time)
		{
			jump_state = JPS_AIR;
			jump_time = 0;
		}
	}
	break;

	case JPS_AIR:
	{

		set.length = .15f;
		set.f0_force = air_force;
		if (/* (leg[LEFT].is_offground == false && leg[RIGHT].is_offground == false) || */ /* (leg[LEFT].d_L0 + leg[RIGHT].d_L0) / 2 */ jump_time >= air_time)
		{
			jump_state = JPS_END;
			jump_time = 0;
		}
	}
	break;

	case JPS_END:
	{
		if ((leg[LEFT].is_offground == false && leg[RIGHT].is_offground == false) || jump_time >= end_time)
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
// 		float tor_fly_l = 200 * tool::annular_err_min(0, ChassisNowState.leg[LEFT].theta, __TOOL_2PI) + 20 * (0 - LegPos_L.d_agl);
// 		float tor_fly_r = 200 * tool::annular_err_min(0, ChassisNowState.leg[RIGHT].theta, __TOOL_2PI) + 20 * (0 - LegPos_R.d_agl);
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
// 		float tor_fly_l = 200 * tool::annular_err_min(0, ChassisNowState.leg[LEFT].theta, __TOOL_2PI) + 10 * (0 - LegPos_L.d_agl);
// 		float tor_fly_r = 200 * tool::annular_err_min(0, ChassisNowState.leg[RIGHT].theta, __TOOL_2PI) + 10 * (0 - LegPos_R.d_agl);
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
