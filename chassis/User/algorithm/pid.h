/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H

#include "struct_typedef.h"

typedef struct
{
  // PID 参数
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;  // 最大输出
  fp32 max_iout; // 最大积分输出

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];  // 微分项 0最新 1上一次 2上上次
  fp32 error[3]; // 误差项 0最新 1上一次 2上上次

  float err_up;
  int cnt_ki;
  int max_cnt_ki;
} PID_Typedef;

void PID_init(PID_Typedef *pid, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout);

/// @brief @brief PID 位置式（普通式）
/// @param pid
/// @param set 目标值
/// @param ref 观测值
/// @return
fp32 PID_Update(PID_Typedef *pid, fp32 set, fp32 ref);

/// @brief PID 差分式
/// @param pid
/// @param set 目标值
/// @param ref 观测值
/// @return
fp32 PID_Diff_Update(PID_Typedef *pid, fp32 set, fp32 ref);

/// @brief pid 输出清除
/// @param pid
void PID_clear(PID_Typedef *pid);

#endif
