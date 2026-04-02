/**
 * @file vmc-dm.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-19
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "vmc-dm.h"

void VMC_Init(VMC_t *vmc) // 给杆长赋值
{
	vmc->l5 = 0.15f; // AE长度 //单位为m
	vmc->l1 = 0.15f; // 单位为m
	vmc->l2 = 0.25f; // 单位为m
	vmc->l3 = 0.25f; // 单位为m
	vmc->l4 = 0.15f; // 单位为m

	vmc->first_flag = 0;

	vmc->is_offground = false;
#ifdef OFFGROUND_FILTER_ENABLE
	Filter_Average_Init(&vmc->filter, 10);
#endif
}

void VMC_5bar_FK(VMC_t *vmc, float phi1, float phi4, float pitch, float dpitch, float dt) // 计算theta和d_theta给lqr用，同时也计算腿长L0
{
	vmc->phi1 = phi1;
	vmc->phi4 = phi4;

	vmc->YD = vmc->l4 * SINF(vmc->phi4);		   // D的y坐标
	vmc->YB = vmc->l1 * SINF(vmc->phi1);		   // B的y坐标
	vmc->XD = vmc->l5 + vmc->l4 * COSF(vmc->phi4); // D的x坐标
	vmc->XB = vmc->l1 * COSF(vmc->phi1);		   // B的x坐标

	vmc->lBD = sqrt((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

	vmc->A0 = 2 * vmc->l2 * (vmc->XD - vmc->XB);
	vmc->B0 = 2 * vmc->l2 * (vmc->YD - vmc->YB);
	vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;
	vmc->phi2 = 2 * atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), vmc->A0 + vmc->C0);
	vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * SINF(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * COSF(vmc->phi2));
	// C点直角坐标
	vmc->XC = vmc->l1 * COSF(vmc->phi1) + vmc->l2 * COSF(vmc->phi2);
	vmc->YC = vmc->l1 * SINF(vmc->phi1) + vmc->l2 * SINF(vmc->phi2);
	// C点极坐标
	vmc->L0 = sqrt((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);

	vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f)); // phi0用于计算lqr需要的theta
	vmc->alpha = PI / 2.0f - vmc->phi0;

	if (vmc->first_flag == 0)
	{
		vmc->last_phi0 = vmc->phi0;
		vmc->first_flag = 1;
	}
	vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
	vmc->d_alpha = 0.0f - vmc->d_phi0;

	// 计算 theta
	vmc->last_theta = vmc->theta;
	vmc->theta = PI / 2.0f - pitch - vmc->phi0; // 得到状态变量1
	vmc->d_theta = (-dpitch - vmc->d_phi0);		// 得到状态变量2
	// vmc->d_theta = (vmc->theta - vmc->last_theta) / dt;
	vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
	vmc->last_d_theta = vmc->d_theta;

	vmc->last_phi0 = vmc->phi0;

	vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;		// 腿长L0的一阶导数
	vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt; // 腿长L0的二阶导数

	vmc->last_d_L0 = vmc->d_L0;
	vmc->last_L0 = vmc->L0;
}

// 计算期望的关节输出力矩
void VMC_5bar_IK(VMC_t *vmc, float tp, float f0)
{
	vmc->Tp = tp;
	vmc->F0 = f0;

	vmc->j11 = (vmc->l1 * SINF(vmc->phi0 - vmc->phi3) * SINF(vmc->phi1 - vmc->phi2)) / SINF(vmc->phi3 - vmc->phi2);
	vmc->j12 = (vmc->l1 * COSF(vmc->phi0 - vmc->phi3) * SINF(vmc->phi1 - vmc->phi2)) / (vmc->L0 * SINF(vmc->phi3 - vmc->phi2));
	vmc->j21 = (vmc->l4 * SINF(vmc->phi0 - vmc->phi2) * SINF(vmc->phi3 - vmc->phi4)) / SINF(vmc->phi3 - vmc->phi2);
	vmc->j22 = (vmc->l4 * COSF(vmc->phi0 - vmc->phi2) * SINF(vmc->phi3 - vmc->phi4)) / (vmc->L0 * SINF(vmc->phi3 - vmc->phi2));

	vmc->torque_set[FRONT] = vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp; // 得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力
	vmc->torque_set[BACK] = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp;  // 得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩
}

#define OFFGROUND_FN_THRESHOLD 0.0f // 离地支持力阈值
#define OFFGROUND_A_RATIO -0.6f		 // 加速度比例

bool OffGround_Detection(VMC_t *leg, float az)
{
	float fn = leg->F0 * COSF(leg->theta) +
			   leg->Tp * SINF(leg->theta) / leg->L0 +
			   -0.6f * (az +
						(-leg->dd_L0 * COSF(leg->theta)) +
						2.0f * leg->d_L0 * leg->d_theta * SINF(leg->theta) +
						leg->L0 * leg->dd_theta * SINF(leg->theta) +
						leg->L0 * leg->d_theta * leg->d_theta * COSF(leg->theta));

#ifdef OFFGROUND_FILTER_ENABLE
	fn = Filter_Average_Update(&leg->filter, fn);
#endif

	leg->fn = fn;

	leg->is_offground = leg->fn < OFFGROUND_FN_THRESHOLD;

	return leg->is_offground;
}
