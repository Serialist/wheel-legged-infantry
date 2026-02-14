#include "vmc-dm.h"
#include "arm_math.h"
#include "chassismotor.h"

extern Robo_Attitude_t att;

void VMC_Init(VMC_t *vmc) // 给杆长赋值
{
	vmc->l5 = 0.15f; // AE长度 //单位为m
	vmc->l1 = 0.15f; // 单位为m
	vmc->l2 = 0.25f; // 单位为m
	vmc->l3 = 0.25f; // 单位为m
	vmc->l4 = 0.15f; // 单位为m

	vmc->first_flag = 0;

	vmc->is_offground = false;
	Filter_Average_Init(&vmc->filter, 10);
}

void VMC_calc_1(VMC_t *vmc, float dt) // 计算theta和d_theta给lqr用，同时也计算腿长L0
{
	static float PitchR = 0.0f;
	static float PithGyroR = 0.0f;
	PitchR = att.pitch;
	PithGyroR = att.pitchspd;

	vmc->YD = vmc->l4 * arm_sin_f32(vmc->phi4);			  // D的y坐标
	vmc->YB = vmc->l1 * arm_sin_f32(vmc->phi1);			  // B的y坐标
	vmc->XD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4); // D的x坐标
	vmc->XB = vmc->l1 * arm_cos_f32(vmc->phi1);			  // B的x坐标

	vmc->lBD = sqrt((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

	vmc->A0 = 2 * vmc->l2 * (vmc->XD - vmc->XB);
	vmc->B0 = 2 * vmc->l2 * (vmc->YD - vmc->YB);
	vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;
	vmc->phi2 = 2 * atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), vmc->A0 + vmc->C0);
	vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * arm_sin_f32(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * arm_cos_f32(vmc->phi2));
	// C点直角坐标
	vmc->XC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
	vmc->YC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);
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

	vmc->last_theta = vmc->theta;
	vmc->theta = PI / 2.0f - PitchR - vmc->phi0; // 得到状态变量1
	vmc->d_theta = (-PithGyroR - vmc->d_phi0);	 // 得到状态变量2
	// vmc->d_theta = (vmc->theta - vmc->last_theta) / dt;

	vmc->last_phi0 = vmc->phi0;

	vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;		// 腿长L0的一阶导数
	vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt; // 腿长L0的二阶导数

	vmc->last_d_L0 = vmc->d_L0;
	vmc->last_L0 = vmc->L0;

	vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
	vmc->last_d_theta = vmc->d_theta;
}

void VMC_calc_2(VMC_t *vmc) // 计算期望的关节输出力矩
{
	vmc->j11 = (vmc->l1 * arm_sin_f32(vmc->phi0 - vmc->phi3) * arm_sin_f32(vmc->phi1 - vmc->phi2)) / arm_sin_f32(vmc->phi3 - vmc->phi2);
	vmc->j12 = (vmc->l1 * arm_cos_f32(vmc->phi0 - vmc->phi3) * arm_sin_f32(vmc->phi1 - vmc->phi2)) / (vmc->L0 * arm_sin_f32(vmc->phi3 - vmc->phi2));
	vmc->j21 = (vmc->l4 * arm_sin_f32(vmc->phi0 - vmc->phi2) * arm_sin_f32(vmc->phi3 - vmc->phi4)) / arm_sin_f32(vmc->phi3 - vmc->phi2);
	vmc->j22 = (vmc->l4 * arm_cos_f32(vmc->phi0 - vmc->phi2) * arm_sin_f32(vmc->phi3 - vmc->phi4)) / (vmc->L0 * arm_sin_f32(vmc->phi3 - vmc->phi2));

	vmc->torque_set[0] = vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp; // 得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力
	vmc->torque_set[1] = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp; // 得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩
}

#define GROUND_DETECTION_THRESHOLD 15.0f
#define OFFGROUND_A_RATIO -0.6f

bool OffGround_Detection(VMC_t *leg)
{
	float fn = leg->F0 * arm_cos_f32(leg->theta) +
			   leg->Tp * arm_sin_f32(leg->theta) / leg->L0 +
			   -0.6f * (att.az +
						(-leg->dd_L0 * arm_cos_f32(leg->theta)) +
						2.0f * leg->d_L0 * leg->d_theta * arm_sin_f32(leg->theta) +
						leg->L0 * leg->dd_theta * arm_sin_f32(leg->theta) +
						leg->L0 * leg->d_theta * leg->d_theta * arm_cos_f32(leg->theta));

	leg->fn = Filter_Average_Update(&leg->filter, fn);
	leg->is_offground = leg->fn < GROUND_DETECTION_THRESHOLD;

	return leg->is_offground;
}

/* ================================================================ KNN offground detection ================================================================ */

#include <stdint.h>
#include <math.h>
#include <string.h>

/* 私有结构体和函数声明 */
typedef struct
{
	float distance;
	int index;
} DistanceIndex;

static void swap_distance_index(DistanceIndex *a, DistanceIndex *b);
static void bubble_sort(DistanceIndex distances[N_SAMPLES], int size);
static void find_k_neighbors(const float query[N_FEATURES], int indices[K_VALUE], float distances[K_VALUE]);

/* 公共函数实现 */

void standardize_features(const float input[N_FEATURES], float output[N_FEATURES])
{
	for (int i = 0; i < N_FEATURES; i++)
	{
		output[i] = (input[i] - model_mean[i]) / model_std[i];
	}
}

float euclidean_distance(const float a[N_FEATURES], const float b[N_FEATURES])
{
	float sum = 0.0f;

	for (int i = 0; i < N_FEATURES; i++)
	{
		float diff = a[i] - b[i];
		sum += diff * diff;
	}

	return sqrtf(sum);
}

static void swap_distance_index(DistanceIndex *a, DistanceIndex *b)
{
	DistanceIndex temp = *a;
	*a = *b;
	*b = temp;
}

static void bubble_sort(DistanceIndex distances[N_SAMPLES], int size)
{
	for (int i = 0; i < size - 1; i++)
	{
		for (int j = 0; j < size - i - 1; j++)
		{
			if (distances[j].distance > distances[j + 1].distance)
			{
				swap_distance_index(&distances[j], &distances[j + 1]);
			}
		}
	}
}

static void find_k_neighbors(const float query[N_FEATURES], int indices[K_VALUE], float distances[K_VALUE])
{
	DistanceIndex all_distances[N_SAMPLES];

	/* 计算到所有训练样本的距离 */
	for (int i = 0; i < N_SAMPLES; i++)
	{
		all_distances[i].distance = euclidean_distance(query, model_X[i]);
		all_distances[i].index = i;
	}

	/* 排序找到最近的K个邻居 */
	bubble_sort(all_distances, N_SAMPLES);

	/* 提取最近的K个邻居 */
	for (int i = 0; i < K_VALUE; i++)
	{
		indices[i] = all_distances[i].index;
		distances[i] = all_distances[i].distance;
	}
}

void knn_predict(const RobotState *state, DetectionResult *result)
{
	/* 组织特征数组 */
	float features[N_FEATURES] = {
		state->pitch,
		state->pitch_dot,
		state->theta,
		state->theta_dot,
		state->x,
		state->x_dot,
		state->F,
		state->Tp};

	/* 标准化特征 */
	float features_std[N_FEATURES];
	standardize_features(features, features_std);

	/* 找到最近的K个邻居 */
	int neighbors_idx[K_VALUE];
	float neighbors_dist[K_VALUE];
	find_k_neighbors(features_std, neighbors_idx, neighbors_dist);

	/* 投票 */
	int votes = 0;
	for (int i = 0; i < K_VALUE; i++)
	{
		votes += model_y[neighbors_idx[i]];
	}

	/* 计算置信度 */
	float confidence = (float)votes / K_VALUE;

	/* 判断结果 */
	result->prediction = (confidence > THRESHOLD) ? 1 : 0;
	result->confidence = confidence;
	result->min_distance = neighbors_dist[0];

	/* 计算平均距离 */
	float sum_dist = 0.0f;
	for (int i = 0; i < K_VALUE; i++)
	{
		sum_dist += neighbors_dist[i];
	}
	result->avg_distance = sum_dist / K_VALUE;
}

/// 应用函数///
int is_airborne(const RobotState *state)
{
	DetectionResult result;
	knn_predict(state, &result);
	return result.prediction;
}
