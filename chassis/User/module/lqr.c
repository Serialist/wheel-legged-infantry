/**
 * @file lqr.c
 * @author Serialist (ba3pt@qq.com)
 * @brief 王洪玺建模 LQR K 拟合函数
 * @version 0.1.0
 * @date 2026-02-19
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "lqr.h"
#include "lqrcoef.h"

float (*lqr_k_coef[])[12][4] = {&lqr_coe1, &lqr_coe2, &lqr_coe3, &lqr_coe4, &lqr_coe5};
float (*lqr_coe)[12][4] = &lqr_coe1;
float k[2][6];

/************************
 * @brief 多项式拟合，腿长对应 K 矩阵
 *
 * @param[out] k 输出的 LQR K 矩阵
 * @param coe 多项式拟合系数
 * @param len 腿长
 * @return float
 ************************/

/**
 * @brief LQR K 矩阵计算
 *
 * @param x [theta dtheta x dx pitch dpitch]
 * @param u [t tp]
 * @param len leg length
 */
void LQR_Control(float *x, float *u, float len)
{
	int i, j, n;
	float k_[6][2];

	// 根据 theta 选择系数
	int8_t coef_idx = (int)roundf(fabsf(x[0]) / (PI / 12)); // 选择系数
	coef_idx = coef_idx < 0	  ? 0
			   : coef_idx > 4 ? 4
							  : coef_idx; // 限幅
	lqr_coe = lqr_k_coef[coef_idx];

	// 计算 k
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 2; j++)
		{
			n = i * 2 + j;
			k_[i][j] = (*lqr_coe)[n][0] * len + (*lqr_coe)[n][1] * len * len + (*lqr_coe)[n][2] * len * len * len + (*lqr_coe)[n][3];
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

	// 计算 u = Kx
	u[0] = u[1] = 0;
	for (int i = 0; i < 6; i++)
	{
		u[0] += x[i] * k[0][i];
		u[1] += x[i] * k[1][i];
	}
}
