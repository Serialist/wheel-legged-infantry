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

#include "wheel_legged_chassis.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "INS_task.h"
#include "bsp_dwt.h"
#include "dt7.h"
#include "user_lib.h"
#include "vmc-dm.h"
#include "can.h"
#include "debug.h"
#include "filter.h"
#include "motor.h"

extern AK_motor_ctrl_fdb_t motorAK10[6];

Chassis_t chassis;
Robo_Attitude_t att;

PID_Typedef pid_tpl = {0},
			pid_tpr = {0},
			leglength_pid_l = {0},
			leglength_pid_r = {0},
			yaw_pid = {0},
			roll_pid = {0},
			tp_pid = {0};

Wheel_Leg_Target_t set;
VMC_t leg_l, leg_r;

/* ======================== he ======================== */

float turn_t; // yaw轴补偿

float tplqrl;
float tplqrr;
float tlqrl;
float tlqrr;

float lqr_k_l[2][6], lqr_k_r[2][6], xl[6], xr[6];

void ChassisInit(void);
void Chassis_Motor_Transmit(Chassis_t *ch);
void LQR_Control(Chassis_t *ch);
void Control_Get(Chassis_t *ch);
void Clamp(float *in, float min, float max);
void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
void Motor_Enable(void);

void Chassis_Task(void const *argument)
{
	ChassisInit();
	VMC_Init(&leg_l);
	VMC_Init(&leg_r);

	for (;;)
	{
		/* ================================ 状态更新 ================================ */

		Control_Get(&chassis);
		LQR_Control(&chassis);

		/* ================================ 电机控制指令 ================================ */

		Chassis_Motor_Transmit(&chassis);
	}
}

void ChassisInit(void)
{
	PID_init(&leglength_pid_l, PID_POSITION, 400, 0, 9000, 120, 0); // 腿长 left
	PID_init(&leglength_pid_r, PID_POSITION, 400, 0, 9000, 120, 0); // 腿长 right
	PID_init(&yaw_pid, PID_POSITION, -0.1f, 0, -0.5f, 0, 0);		// yaw
	PID_init(&roll_pid, PID_POSITION, 0.8f, 0, 0, 30.0f, 0);		// roll
	PID_init(&tp_pid, PID_POSITION, 1.3, 0, 3, 1.5, 0);				// 劈叉

	// 腿摆角扭矩pid，用于板凳模型
	PID_init(&pid_tpl, PID_POSITION, 80, 0, 400, 10, 0);
	PID_init(&pid_tpr, PID_POSITION, 80, 0, 400, 10, 0);

	set.left_length = set.right_length = 0.15f;

	Motor_Enable();
}

/************************
 * @brief 初始化电机控制
 *
 ************************/
void Motor_Enable(void)
{
	for (int a = 1; a <= 4; a++)
	{
		controller_init(a);
		osDelay(1);
	}
}

RobotState knnstate = {0};
DetectionResult knnrsult = {0};

void chassis_sys_calc(Chassis_t *ch)
{
	leg_l.phi1 = PI / 2.0f - motorAK10[2].angle;
	leg_l.phi4 = PI / 2.0f - motorAK10[3].angle;
	leg_r.phi1 = PI / 2.0f - motorAK10[0].angle;
	leg_r.phi4 = PI / 2.0f - motorAK10[1].angle;

	VMC_calc_1(&leg_l, 3.0f / 1000.0f);
	VMC_calc_1(&leg_r, 3.0f / 1000.0f);

	OffGround_Detection(&leg_l);
	OffGround_Detection(&leg_r);
	chassis.robo_status.flag.above = leg_l.is_offground && leg_r.is_offground;

	knnstate.pitch = att.pitch;				 /* 俯仰角 */
	knnstate.pitch_dot = att.pitchspd;		 /* 俯仰角速度 */
	knnstate.theta = leg_l.theta;			 /* 腿部角度 */
	knnstate.theta_dot = leg_l.d_theta;		 /* 腿部角速度 */
	knnstate.x = chassis.state.x_filter;	 /* 水平位置 */
	knnstate.x_dot = chassis.state.v_filter; /* 水平速度 */
	knnstate.F = leg_l.F0;					 /* 竖直腿部推力 */
	knnstate.Tp = tplqrl;					 /* 髋关节推力 */
	knn_predict(&knnstate, &knnrsult);
}

void Chassis_Motor_Transmit(Chassis_t *ch)
{
	if (rc_ctrl.rc.s[S_R] != DOWN)
	{
		for (int i = 0; i < 6; i++)
			ch->ak_set[i].torset = 0;
	}

	/// @brief 用 3508
	RM_Motor_Transmit(&hcan1, M3508_TX_ID_2,
					  0,
					  HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[4].torset),
					  HEXROLL_TORQUE_TO_CURRENT(ch->ak_set[5].torset),
					  0);
	osDelay(1);

	// MIT模式下发送
	AK_MIT_Transmit(1, 0, 0, 0, 0, ch->ak_set[0].torset);
	AK_MIT_Transmit(3, 0, 0, 0, 0, ch->ak_set[3].torset);
	osDelay(1);
	AK_MIT_Transmit(2, 0, 0, 0, 0, ch->ak_set[1].torset);
	AK_MIT_Transmit(4, 0, 0, 0, 0, ch->ak_set[2].torset);
	osDelay(1);
}

void Control_Get(Chassis_t *ch)
{
	ch->robo_status.last_behavior = ch->robo_status.behavior;

	if (rc_ctrl.rc.s[S_R] == DOWN) // 正常行驶
	{
		set.v = rc_ctrl.rc.ch[L_Y] * 3.5f / 660.0f;
		set.yaw -= rc_ctrl.rc.ch[L_X] * 0.001f;
		set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
		set.roll = -rc_ctrl.rc.ch[R_X] * 45.0f / 660.0f;

		if (set.v != 0)
			set.x = ch->state.x_filter;
	}
	else
	{
		set.v = 0;
		set.yaw = att.toatalyaw;
		set.roll = 0;
		set.left_length = set.right_length = 0.2f;
		set.x = chassis.state.x_filter;
	}
}

/// @brief 限幅
void Clamp(float *in, float min, float max)
{
	if (*in < min)
	{
		*in = min;
	}
	else if (*in > max)
	{
		*in = max;
	}
}

// 支持力前馈
float fn_feedforward = 0;

/// @date 2026-02-09 14:52
float lqr_coe[12][4] = {
	{-267.114936387831278, 661.491964532750558, -682.122523751811173, -4.428510482632967},
	{-2.711097130725094, -373.817462228878014, 658.245716813072590, 35.552038864414683},
	{-16.719034445449012, 0.001400582208879, 2.705005533161179, 0.228167376789969},
	{17.556152854418329, -66.814162026074428, 79.592060370284798, 0.731319298065810},
	{-72.034005903728726, 208.049006752377807, -219.798850931020809, -1.197773146357612},
	{-17.308619529083341, -85.852591464970857, 186.514789606525113, 13.588303095483351},
	{-52.943621177496880, 153.374866823909088, -166.839567698466908, -3.477453334784835},
	{-61.765563243005509, 74.240839845030607, -5.577331606779901, 17.470978929020220},
	{-378.876803975313123, 648.471714531981661, -393.437171515994009, 93.513364783697170},
	{767.233123487312355, -2225.063829490502940, 2325.201976985349120, 79.767017940150794},
	{-24.146756619343769, 43.113614435957103, -30.567485088434012, 6.614411854248947},
	{39.995777317992413, -108.560240305351599, 108.116985537683306, 3.430609614177370}};

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void LQR_Control(Chassis_t *ch)
{
	LQR_K_Calc(lqr_k_l, lqr_coe, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe, leg_r.L0);

	xl[0] = (leg_l.theta);
	xl[1] = (leg_l.d_theta);
	xl[2] = (ch->state.x_filter - set.x);
	xl[3] = (ch->state.v_filter - set.v);
	xl[4] = (att.pitch);
	xl[5] = (att.pitchspd);

	xr[0] = (leg_r.theta);
	xr[1] = (leg_r.d_theta);
	xr[2] = (ch->state.x_filter - set.x);
	xr[3] = (ch->state.v_filter - set.v);
	xr[4] = (att.pitch);
	xr[5] = (att.pitchspd);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (chassis.robo_status.flag.above)
	{
		for (int i = 0; i < 6; i++)
		{
			lqr_k_l[0][i] = 0.0f;
			lqr_k_r[0][i] = 0.0f;
			if (i != 0 && i != 1)
			{
				lqr_k_l[1][i] = 0.0f;
				lqr_k_r[1][i] = 0.0f;
			}
		}
	}

	/// @brief u = - K * x。分左右腿
	tlqrl = tplqrl = tlqrr = tplqrr = 0;
	for (int i = 0; i < 6; i++)
	{
		tlqrl += xl[i] * lqr_k_l[0][i];
		tplqrl += xl[i] * lqr_k_l[1][i];
		
		tlqrr += xr[i] * lqr_k_r[0][i];
		tplqrr += xr[i] * lqr_k_r[1][i];
	}

	/* ================================ 轮 解算 ================================ */
	turn_t = yaw_pid.Kp * (set.yaw - att.toatalyaw) - yaw_pid.Kd * att.yawspd; // 这样计算更稳一点
	if (ch->robo_status.flag.above)
		turn_t = 0;
	set.torque[5] = tlqrl - turn_t;
	set.torque[4] = tlqrr + turn_t;

	/* ================================ 腿 解算 ================================ */

	/// @brief 腿推力 PID
	leg_l.F0 = 55.0f * arm_cos_f32(leg_l.theta) +
			   PID_Calc(&leglength_pid_l, set.left_length, leg_l.L0);
	leg_r.F0 = 55.0f * arm_cos_f32(leg_r.theta) +
			   PID_Calc(&leglength_pid_r, set.right_length, leg_r.L0);

	leg_l.Tp = tplqrl;
	leg_r.Tp = tplqrr;

	/// @brief 正 VMC
	VMC_calc_2(&leg_l);
	VMC_calc_2(&leg_r);

	/// @brief 发送 buf
	set.torque[0] = leg_r.torque_set[0];
	set.torque[1] = leg_r.torque_set[1];
	set.torque[3] = leg_l.torque_set[0];
	set.torque[2] = leg_l.torque_set[1];

/* ================================ 发送 ================================ */

/// @brief 限幅
#define HIP_TORQUE_MAX 40.0f
#define HUB_TORQUE_MAX 2.5f
	Clamp(&set.torque[0], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[1], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[2], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[3], -HIP_TORQUE_MAX, HIP_TORQUE_MAX);
	Clamp(&set.torque[4], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);
	Clamp(&set.torque[5], -HUB_TORQUE_MAX, HUB_TORQUE_MAX);

	/// @brief 发送
	ch->ak_set[0].torset = -set.torque[0];
	ch->ak_set[1].torset = -set.torque[1];
	ch->ak_set[2].torset = set.torque[2];
	ch->ak_set[3].torset = set.torque[3];
	ch->ak_set[4].torset = -set.torque[4];
	ch->ak_set[5].torset = set.torque[5];
}

/************************
 * @brief 多项式拟合，腿长对应 K 矩阵
 *
 * @param[out] k 输出的 LQR K 矩阵
 * @param coe 多项式拟合系数
 * @param len 腿长
 * @return float
 ************************/
void LQR_K_Calc(float k[2][6], float coe[12][4], float len)
{
	int i, j, n;
	float k_[6][2];

	// 计算 k
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 2; j++)
		{
			n = i * 2 + j;
			k_[i][j] = coe[n][0] * len + coe[n][1] * len * len + coe[n][2] * len * len * len + coe[n][3];
		}
	}

	// 转置
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 2; j++)
		{
			k[j][i] = k_[i][j];
		}
	}
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
