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
Robo_State_t robo_state;
Robo_Attitude_t att;
JUMP_State_t jump_state = JS_NONE;

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

/* ======================== he ======================== */

float turn_t; // yaw轴补偿
float tp_phi; // 劈叉
float tplqrl;
float tplqrr;
float tlqrl;
float tlqrr;
// 支持力前馈
float leg_force_l = 0;
float leg_force_r = 0;

float lqr_k_l[2][6], lqr_k_r[2][6], xl[6], xr[6];

void ChassisInit(void);
void Chassis_Motor_Transmit(Chassis_t *ch);
void LQR_Control(Chassis_t *ch);
void Control_Get(Chassis_t *ch);
void Clamp(float *in, float min, float max);
void LQR_K_Calc(float k[2][6], float coe[12][4], float len);
void Motor_Enable(void);
void Jump_FSM(void);

void Chassis_Task(void const *argument)
{
	ChassisInit();
	VMC_Init(&leg_l);
	VMC_Init(&leg_r);

	for (;;)
	{
		/* ================================ 状态更新 ================================ */

		Control_Get(&chassis);
		Jump_FSM();
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
	PID_init(&tp_offground_pid, PID_POSITION, 80, 0, 400, 0, 0);

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

float ttphi1 = 0, ttphi4 = 0, tttp = 0, ttf0 = 0;

void chassis_sys_calc(Chassis_t *ch)
{
	leg_r.phi1 = ttphi1;
	leg_r.phi4 = ttphi4;
	VMC_calc_1(&leg_l, 3.0f / 1000.0f);
	VMC_calc_1(&leg_r, 3.0f / 1000.0f);

	OffGround_Detection(&leg_l);
	OffGround_Detection(&leg_r);
	if (jump_state != JS_NONE &&
		jump_state != JS_INIT)
		chassis.robo_status.flag.above = true;
	else
		// chassis.robo_status.flag.above = leg_l.is_offground && leg_r.is_offground;
		chassis.robo_status.flag.above = false;
}

void Chassis_Motor_Transmit(Chassis_t *ch)
{
	if (rc_ctrl.rc.s[S_L] == UP)
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
	AK_MIT_Transmit(2, 0, 0, 0, 0, ch->ak_set[1].torset);
	osDelay(1);
	AK_MIT_Transmit(3, 0, 0, 0, 0, ch->ak_set[3].torset);
	AK_MIT_Transmit(4, 0, 0, 0, 0, ch->ak_set[2].torset);
	osDelay(1);
}

uint8_t last_switch = 0;

void Control_Get(Chassis_t *ch)
{
	if (rc_ctrl.rc.s[S_L] == MID || last_switch == DOWN) // 正常行驶
	{
		robo_state = RBS_RUN;
		set.v = rc_ctrl.rc.ch[L_Y] * 3.5f / 660.0f;
		set.yaw -= rc_ctrl.rc.ch[L_X] * 0.001f;
		set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
		set.roll = -rc_ctrl.rc.ch[R_X] * 45.0f / 660.0f;

		if (set.v != 0)
			set.x = ch->state.x_filter;
	}
	else if (rc_ctrl.rc.s[S_L] == DOWN && jump_state == JS_NONE)
	{
		set.v = rc_ctrl.rc.ch[L_Y] * 3.5f / 660.0f;
		set.yaw -= rc_ctrl.rc.ch[L_X] * 0.001f;
		set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
		set.roll = -rc_ctrl.rc.ch[R_X] * 45.0f / 660.0f;

		if (set.v != 0)
			set.x = ch->state.x_filter;
		robo_state = RBS_JUMP;
	}
	else
	{
		set.v = 0;
		set.yaw = att.toatalyaw;
		set.roll = 0;
		set.left_length = set.right_length = 0.2f;
		set.x = chassis.state.x_filter;
	}
	last_switch = rc_ctrl.rc.s[S_L];
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

/// @date 2026-02-09 14:52
/// @date 2026-02-15 15:56

float lqr_coe1[12][4] = {
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

float lqr_coe2[12][4] = {
	{-230.364672195870412, 606.207569022922939, -655.864426389296341, -6.998026652757481},
	{-100.611059675673104, -53.714394604406777, 309.909633631296913, 42.418636554139248},
	{-20.222861207522651, 3.153626015960692, -1.475811766789321, 0.347236295465262},
	{16.768774759607329, -69.126444810256260, 86.015912058565178, 0.907005251998973},
	{-74.570168636066299, 228.458915635308102, -251.624245400588904, -1.959729430960110},
	{-48.206879356691402, 17.558447147795860, 72.099068741946880, 15.923526950692240},
	{-68.505392533739226, 203.348352754011898, -225.539607576592488, -3.323418655042467},
	{-69.887906671399989, 91.338177028783605, -14.774596110553970, 18.371871508035529},
	{-466.647282157123698, 961.815613840455171, -745.966139966108017, 97.094055471253697},
	{828.472248914999113, -2557.669361855274019, 2799.972802728771967, 80.532377693908572},
	{-26.655428572747610, 50.110746331662448, -36.930511530741533, 6.836838712804244},
	{43.498178717290422, -125.222971235698296, 130.549508604835012, 3.381132339965290}};

float lqr_coe3[12][4] = {
	{-199.865458234014000, 575.132458387696715, -666.372144803497235, -10.182087685111750},
	{-271.921401180376790, 567.347582445559624, -419.723623151471600, 53.394539557795319},
	{-25.631704698046558, 2.995884420910934, -5.934063203682094, 0.485216495068803},
	{10.747808233993499, -53.986942535698986, 74.140141527714988, 1.436174299190149},
	{-72.537381513554976, 239.268249648506696, -277.264863055496392, -3.474439174065291},
	{-107.018994557352798, 235.934737173134209, -188.514593254159507, 19.754895289839371},
	{-85.708875638789436, 247.599470066270300, -276.130049611867889, -3.353618910275145},
	{-101.389409834953696, 205.613483482910794, -146.963789856112413, 20.372529079870329},
	{-635.186959229445279, 1633.283008451780915, -1564.328524901985929, 103.057532137609499},
	{879.935289035790561, -2960.760485394492207, 3449.711157496796204, 86.330006281981042},
	{-31.714710855418179, 66.637675673222375, -54.432265148993892, 7.280444669264733},
	{48.759108615806568, -152.061382100039793, 168.319274065229592, 3.332454855842162}};

float lqr_coe4[12][4] = {
	{-186.063938203350602, 555.967630491363252, -674.353567258832300, -13.102034579992850},
	{-406.239360506138325, 1122.883460884366059, -1133.029408087488036, 60.370356578884397},
	{-29.822647014180870, -3.872856500962053, -9.227129693600792, 0.527397397990237},
	{2.023246282438108, -22.204988729956689, 38.222039957818289, 1.991350453420185},
	{-66.025159475804358, 228.296654713604198, -273.520193501379708, -4.865627246998940},
	{-152.423875298702598, 427.440677792708925, -437.366432383137408, 22.059557422719831},
	{-86.082622321934807, 236.150309101375910, -266.844642874237081, -3.969844099539052},
	{-137.411437622090887, 365.088620453300280, -357.030276777286417, 21.700581435058581},
	{-768.560608910246856, 2246.292650360179778, -2385.091146542260958, 106.078689127240494},
	{841.120224915519316, -2986.620076337565024, 3618.955508004518833, 96.929335676945101},
	{-37.509234652571308, 88.729588338142662, -81.158105866934051, 7.828674905700353},
	{52.464325726171957, -173.019142377427187, 199.756836984305210, 3.323037510577107}};

float lqr_coe5[12][4] = {
	{-184.458936530901894, 531.884892553581835, -661.300416145146528, -15.285211385888250},
	{-472.776732485589889, 1433.036099243993931, -1558.830426912259099, 62.969616729955227},
	{-32.716594471732883, -21.313689281508481, -9.276883043466574, 0.544322138121068},
	{-3.052910666957877, 0.955768856616226, 9.657289389693435, 2.245645441550296},
	{-59.768636107660733, 211.553570515085312, -257.755245297897204, -5.824584648419600},
	{-174.355418613547187, 531.397976469802870, -581.861326368796995, 22.846248924073969},
	{-77.047121457948307, 200.325308382910606, -232.415978924297804, -4.705935105596997},
	{-155.701618018759405, 463.015810055513896, -497.551843785064875, 21.641437634987110},
	{-828.508374160839480, 2570.713405059018896, -2856.624105325253822, 105.901324857637704},
	{776.012547619069210, -2826.466299415235881, 3490.138528212020901, 105.945255257910304},
	{-41.013708475383638, 103.469315444135304, -100.253042731912601, 8.355268349294493},
	{54.833161702719337, -185.631927264598602, 218.706931724528488, 3.234501659370782}};

float (*lqr_k_coef[])[12][4] = {&lqr_coe1, &lqr_coe2, &lqr_coe3, &lqr_coe4, &lqr_coe5};
float (*lqr_coe)[12][4] = &lqr_coe1;

/***********************************************
 * @brief 平衡行驶过程(左右两腿分别进行LQR运算)
 *
 * @param ch
 *************************************************/
void LQR_Control(Chassis_t *ch)
{
	int coef_idx = (int)roundf(fabs(leg_l.theta) / (PI / 12));
	coef_idx = coef_idx < 0	  ? 0
			   : coef_idx > 4 ? 4
							  : coef_idx;
	lqr_coe = lqr_k_coef[coef_idx];

	LQR_K_Calc(lqr_k_l, lqr_coe1, leg_l.L0);
	LQR_K_Calc(lqr_k_r, lqr_coe1, leg_r.L0);

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

		// tlqrl = tlqrr = 0;
		// tplqrl = tp_offground_pid.Kp * leg_l.theta + tp_offground_pid.Kd * leg_l.d_theta;
		// tplqrr = tp_offground_pid.Kp * leg_r.theta + tp_offground_pid.Kd * leg_r.d_theta;
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

	// set.left_length = set.height / arm_cos_f32(leg_l.theta);
	// set.right_length = set.height / arm_cos_f32(leg_r.theta);

	/// @brief 腿推力 PID
	leg_l.F0 = 55.0f * arm_cos_f32(leg_l.theta) +
			   PID_Calc(&leglength_pid_l, set.left_length, leg_l.L0) +
			   leg_force_l;
	leg_r.F0 = 55.0f * arm_cos_f32(leg_r.theta) +
			   PID_Calc(&leglength_pid_r, set.right_length, leg_r.L0) +
			   leg_force_r;

	leg_l.Tp = tplqrl;
	leg_r.Tp = tplqrr;

	leg_r.Tp = tttp;
	leg_r.F0 = ttf0;

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

uint32_t jump_time = 0;
// ms
uint32_t stretch_time = 200;
uint32_t shrink_time = 200;
uint32_t air_time = 200;
uint32_t end_time = 100;
// N
float stretch_force = 150;
float shrink_force = -150;
float air_force = 50;

void Jump_FSM(void)
{
	switch (jump_state)
	{
	case JS_NONE:
	{
		leg_force_l = leg_force_r = 0;
		set.left_length = set.right_length = (rc_ctrl.rc.ch[R_Y] * 0.01f) / 66 + 0.2f;
		if (robo_state == RBS_JUMP)
		{
			jump_state = JS_INIT;
			jump_time = 0;
		}
	}
	break;

	case JS_INIT:
	{
		set.left_length = set.right_length = .15f;
		if (jump_time >= 500)
		{
			jump_state = JS_STRETCH;
			jump_time = 0;
		}
	}
	break;

	case JS_STRETCH:
	{
		set.left_length = set.right_length = .4f;
		leg_force_l = leg_force_r = stretch_force;
		if (((leg_l.L0 + leg_r.L0) / 2) >= .3f || jump_time >= stretch_time)
		{
			jump_state = JS_SHRINK;
			jump_time = 0;
		}
	}
	break;

	case JS_SHRINK:
	{
		set.left_length = set.right_length = .15f;
		leg_force_l = leg_force_r = shrink_force;
		if (((leg_l.L0 + leg_r.L0) / 2) <= .18f || jump_time >= shrink_time)
		{
			jump_state = JS_AIR;
			jump_time = 0;
		}
	}
	break;

	case JS_AIR:
	{

		set.left_length = set.right_length = .2f;
		leg_force_l = leg_force_r = air_force;
		if (/* (leg_l.is_offground == false && leg_r.is_offground == false) || */ /* (leg_l.d_L0 + leg_r.d_L0) / 2 */ jump_time >= air_time)
		{
			jump_state = JS_END;
			jump_time = 0;
		}
	}
	break;

	case JS_END:
	{
		if ((leg_l.is_offground == false && leg_r.is_offground == false) || jump_time >= end_time)
		{
			jump_state = JS_NONE;
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
