/************************
 * @file pid.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-09
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "pid.h"
#include "main.h"
#include "user_lib.h"
#include "math.h"

/**
 * @brief pid struct data init
 *
 * @param pid PID结构数据指针
 * @param mode PID_POSITION:普通PID
 *             PID_DELTA: 差分PID
 * @param kp
 * @param ki
 * @param kd
 * @param max_out 最大输出
 * @param max_iout 最大积分输出
 */
void PID_init(PID_Typedef *pid, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief PID 位置式（普通式）
 *
 * @param pid
 * @param set
 * @param ref
 * @return fp32
 */
fp32 PID_Update(PID_Typedef *pid, fp32 set, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    pid->Pout = pid->Kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    ClampAbsf(pid->Iout, pid->max_iout);

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    ClampAbsf(pid->out, pid->max_out);

    return pid->out;
}

/**
 * @brief PID 差分式
 *
 * @param pid
 * @param set
 * @param ref
 * @return fp32
 */
fp32 PID_Diff_Update(PID_Typedef *pid, fp32 set, fp32 ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    ClampAbsf(pid->out, pid->max_out);

    return pid->out;
}

/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(PID_Typedef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
