/**
 * @file vmc-dm.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-13
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef VMC_CALC_H
#define VMC_CALC_H

// #define OFFGROUND_FILTER_ENABLE

#include "user_lib.h"

#ifdef OFFGROUND_FILTER_ENABLE
#include "filter.h"
#endif

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5; // AE长度 //单位为m
	float l1; // 单位为m
	float l2; // 单位为m
	float l3; // 单位为m
	float l4; // 单位为m

	float XB, YB; // B点的坐标
	float XD, YD; // D点的坐标

	float XC, YC;	// C点的直角坐标
	float L0, phi0; // C点的极坐标

	float alpha;
	float d_alpha;

	float lBD; // BD两点的距离

	float d_phi0;	 // 现在C点角度phi0的变换率
	float last_phi0; // 上一次C点角度，用于计算角度phi0的变换率d_phi0

	float A0, B0, C0; // 中间变量
	float phi2, phi3;
	float phi1, phi4;

	float j11, j12, j21, j22; // 笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float torque_set[2];

	float F0;
	float Tp;

	float theta;
	float last_theta;
	float d_theta; // theta的一阶导数
	float last_d_theta;
	float dd_theta; // theta的二阶导数

	float d_L0;	 // L0的一阶导数
	float dd_L0; // L0的二阶导数
	float last_L0;
	float last_d_L0;

	uint8_t first_flag;
	uint8_t leg_flag; // 腿长完成标志

	float fn; // 支持力
	float is_offground;

#ifdef OFFGROUND_FILTER_ENABLE
	Filter_Average_t filter;
#endif
} VMC_t;

void VMC_Init(VMC_t *vmc);																   // 给杆长赋值
void VMC_5bar_FK(VMC_t *vmc, float phi1, float phi4, float pitch, float dpitch, float dt); // 正运动学，pitch，vpitch，
void VMC_5bar_IK(VMC_t *vmc, float tp, float f0);										   // 逆运动学
bool OffGround_Detection(VMC_t *leg, float az);											   // 离地检测

#endif
