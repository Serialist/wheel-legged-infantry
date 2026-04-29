/**
 * @file observer.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-01-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

/* ================================================================ include ================================================================ */

#include "cmsis_os.h"

#include "kalman_filter_whx.h"
#include "rm_motor.h"
#include "vmc-dm.h"

#include "chassis.hpp"
#include "ins-task.hpp"
#include "observer.hpp"

/* ================================================================ macro ================================================================ */

#define TASK_PERIOD_MS 3 // 任务周期

/* ================================================================ typedef ================================================================ */

/* ================================================================ variable ================================================================ */

whxKalmanFilter_t vaEstimateKF; // 这是一个卡尔曼滤波器对象

float vaEstimateKF_F[4] = {
	1.0f, 0.003f, 0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f}; // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 0.0f, 0.5f}; // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f, 0.0f, 100.0f};

const float vaEstimateKF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f}; // 设置矩阵H为常量

extern VMC_t leg[2];
extern Wheel_Leg_Target_t set;

float vel_acc[2];

float motor_vel_r;
float motor_vel_l;

float wr, wl = 0.0f;		  // wheel 轮毂速度
float vrb = 0.0f, vlb = 0.0f; // vehicle
float aver_v = 0.0f;

extern RM_Motor_Feedback_t m3508[2];

Observer_t ob;

/* ================================================================ prototype ================================================================ */

void ObEKF_Init();
void ObEKF_Update(float acc, float vel);

/* ================================================================ function ================================================================ */

/************************************************
 * @brief os task
 *
 * @param argument
 ********************************/
extern "C" void Observer_Task(void const *argument)
{
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	ObEKF_Init();

	while (1)
	{
		uint32_t current_time_ms = HAL_GetTick();

		motor_vel_r = HEXROLL_VELOCITY(&m3508[RIGHT]);
		motor_vel_l = -HEXROLL_VELOCITY(&m3508[LEFT]);

		// 机体速度观测器

		// 右
		wr =					  //
			motor_vel_r			  // 轮毂反馈速度
			+ ins.Gyro[0]		  // 机体角速度
			+ leg[RIGHT].d_alpha; // 腿速度
		vrb =
			wr * WHEEL_RADIUS
			+ leg[RIGHT].L0 * leg[RIGHT].d_theta * arm_cos_f32(leg[RIGHT].theta)
			+ leg[RIGHT].d_L0 * arm_sin_f32(leg[RIGHT].theta); // 右机体速度

		// 左
		wl = motor_vel_l + ins.Gyro[0] + leg[LEFT].d_alpha; // 左轮速度
		vlb = wl * WHEEL_RADIUS
			  + leg[LEFT].L0 * leg[LEFT].d_theta * arm_cos_f32(leg[LEFT].theta)
			  + leg[LEFT].d_L0 * arm_sin_f32(leg[LEFT].theta); // 左机体速度

		// 总体互补滤波
		aver_v = (vrb + vlb) / 2.0f; // 取平均

		ObEKF_Update(ins.Accel[1], aver_v); // ins 加速度 轮毂反馈速度 融合滤波

		// 原地自转的过程中v_filter和x_filter应该都是为0
		ob.v = vel_acc[0];

		ob.x += ob.v * (TASK_PERIOD_MS * 0.001f);

		// 精确周期控制
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
	}
}

// 达妙例程速度观测器//

void ObEKF_Init()
{
	whxKalman_Filter_Init(
		&vaEstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

	memcpy(vaEstimateKF.F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
	memcpy(vaEstimateKF.P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
	memcpy(vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
	memcpy(vaEstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	memcpy(vaEstimateKF.H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void ObEKF_Update(float acc, float vel)
{
	// 卡尔曼滤波器测量值更新
	vaEstimateKF.MeasuredVector[0] = vel; // 测量速度
	vaEstimateKF.MeasuredVector[1] = acc; // 测量加速度

	// 卡尔曼滤波器更新函数
	whxKalman_Filter_Update(&vaEstimateKF);

	// 提取估计值
	for (uint8_t i = 0; i < 2; i++)
	{
		vel_acc[i] = vaEstimateKF.FilteredValue[i];
	}
}
