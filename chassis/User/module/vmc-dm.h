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

#include "user_lib.h"
#include "filter.h"

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
	Filter_Average_t filter;
} VMC_t;

void VMC_Init(VMC_t *vmc);			   // 给杆长赋值
void VMC_calc_1(VMC_t *vmc, float dt); // 计算theta和d_theta给lqr用，同时也计算腿长L0
void VMC_calc_2(VMC_t *vmc);		   // 计算期望的关节输出力矩
bool OffGround_Detection(VMC_t *leg);

/* ================================================ KNN offground detection ================================================ */

/* 模型配置 */
#define K_VALUE 3
#define N_SAMPLES 10
#define N_FEATURES 8
#define THRESHOLD 0.75f

/* ================================ simple ================================ */

/* 从MATLAB训练结果复制的模型参数，根据数据数量可以进行修改*/

/* 标准化参数 - 均值 */
static const float model_mean[N_FEATURES] = {
	0.005973f,	/* pitch */
	-0.103929f, /* pitch_dot */
	0.002233f,	/* theta */
	0.177344f,	/* theta_dot */
	-0.058028f, /* x */
	-0.027760f, /* x_dot */
	47.847920f, /* F */
	-0.606468f	/* Tp */
};

/* 标准化参数 - 标准差 */
static const float model_std[N_FEATURES] = {
	0.002337f, /* pitch */
	0.339937f, /* pitch_dot */
	0.004140f, /* theta */
	0.481610f, /* theta_dot */
	0.002149f, /* x */
	0.044313f, /* x_dot */
	1.414255f, /* F */
	3.670786f  /* Tp */
};

/* 标准化训练数据 */
static const float model_X[N_SAMPLES][N_FEATURES] = {
	{1.781504f, 1.084889f, -1.527304f, -1.119586f, -0.819735f, -0.266070f, 1.007166f, -0.771141f},
	{-0.252753f, -0.673127f, -0.487970f, 0.491739f, -0.405681f, -0.505276f, 1.086728f, -0.729060f},
	{-1.076723f, 0.627366f, 1.240710f, -0.654354f, -0.628991f, 1.312512f, -0.897441f, 1.243179f},
	{1.781504f, 1.084889f, -0.335802f, -0.918199f, -0.819735f, -0.266070f, -0.946187f, 1.131216f},
	{-0.205265f, 0.464416f, 0.768989f, -0.421137f, -0.605729f, 0.802912f, -1.217977f, 1.305815f},
	{-0.252753f, -0.673127f, 0.632520f, 0.491411f, -0.405681f, -0.505276f, -0.898989f, 0.686460f},
	{-1.076723f, 0.627366f, 1.144095f, -0.754352f, 1.231929f, 1.312512f, 0.670289f, -0.648783f},
	{-0.205265f, 0.464416f, -0.526930f, -0.320981f, -0.605729f, 0.802912f, 0.943019f, -1.351147f},
	{-0.246763f, -1.503544f, -1.399289f, 1.552483f, 1.529676f, -1.344078f, 0.975708f, -0.787055f},
	{-0.246763f, -1.503544f, 0.490979f, 1.652977f, 1.529676f, -1.344078f, -0.722317f, -0.079485f}};

/* 训练标签 (1=离地, 0=着地) */
static const uint8_t model_y[N_SAMPLES] = {
	1, 1, 0, 0, 0, 0, 1, 1, 1, 0};

/* ================================ api ================================ */

/* 机器人状态结构体 */
typedef struct
{
	float pitch;	 /* 俯仰角 */
	float pitch_dot; /* 俯仰角速度 */
	float theta;	 /* 腿部角度 */
	float theta_dot; /* 腿部角速度 */
	float x;		 /* 水平位置 */
	float x_dot;	 /* 水平速度 */
	float F;		 /* 竖直腿部推力 */
	float Tp;		 /* 髋关节推力 */
} RobotState;

/* 检测结果结构体 */
typedef struct
{
	int prediction;		/* 预测结果: 0=着地, 1=离地 */
	float confidence;	/* 置信度 */
	float min_distance; /* 最近距离 */
	float avg_distance; /* 平均距离 */
} DetectionResult;

/* 函数声明 */

/**
 * @brief 标准化特征向量
 * @param input 输入特征数组
 * @param output 输出标准化后的数组
 */
void standardize_features(const float input[N_FEATURES], float output[N_FEATURES]);

/**
 * @brief 计算欧氏距离
 * @param a 向量A
 * @param b 向量B
 * @return 欧氏距离
 */
float euclidean_distance(const float a[N_FEATURES], const float b[N_FEATURES]);

/**
 * @brief KNN预测主函数
 * @param state 机器人状态
 * @param result 检测结果
 */
void knn_predict(const RobotState *state, DetectionResult *result);

/**
 * @brief 简化KNN预测（返回是否离地）
 * @param state 机器人状态
 * @return 1=离地, 0=着地
 */
int is_airborne(const RobotState *state);

#endif
