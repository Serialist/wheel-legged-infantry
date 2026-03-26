#ifndef __CHASSISMOTOR_H__
#define __CHASSISMOTOR_H__

#include "stdint.h"
#include "user_lib.h"
#include "dt7.h"
#include "main.h"
#include "struct_typedef.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define V_MIN_2 -45.0f
#define V_MAX_2 45.0f
#define T_MIN -65.0f
#define T_MAX 65.0f
#define T_MIN_2 -15.0f
#define T_MAX_2 15.0f
#define Kp_MIN 0
#define Kp_MAX 500.0f
#define Kd_MIN 0
#define Kd_MAX 5.0f

#define wheelRadius 0.066f

typedef struct
{
	float prev_theta; // 上次角度
	float v_prev;	  // 前次速度
	float v_prev2;	  // 前前次速度
	float b0, a1, a2; // 滤波器系数
} SpeedEstimator;

typedef struct
{
	float motor_pos;   // 电机位置
	float motor_spd;   // 电机速度
	float motor_cur;   // 电机电流
	float motor_temp;  // 电机温度
	float motor_error; // 电机故障码
} AK_motor_fdb_t;

typedef struct
{
	float angle;		   // 电机位置
	float motor_ctrlspd;   // 电机速度
	float motor_ctrltor;   // 电机扭矩
	float motor_ctrltemp;  // 电机温度
	float motor_ctrlerror; // 电机故障码
} AK_motor_ctrl_fdb_t;

typedef enum
{
	CAN_PACKET_SET_DUTY = 0,	  // 占空比模式
	CAN_PACKET_SET_CURRENT,		  // 电流环模式
	CAN_PACKET_SET_CURRENT_BRAKE, // 电流刹车模式
	CAN_PACKET_SET_RPM,			  // 转速模式
	CAN_PACKET_SET_POS,			  // 位置模式
	CAN_PACKET_SET_ORIGIN_HERE,	  // 设置原点模式
	CAN_PACKET_SET_POS_SPD,		  // 位置速度环模式
} CAN_PACKET_ID;

typedef struct
{

	float spdset;
	float posset;
	float torset;

} CM_TRANSMIT_DATA;

// 伺服模式//
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);
// 运控模式//

void AK_Motor_MIT_Enable(uint8_t id);
void AK_Motor_MIT_Disable(uint8_t id);
void AK_Motor_MIT_Setorigin(uint8_t id);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
void AK_Motor_MIT_Transmit(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff);

/* ================================ new cubemars code ================================ */

typedef struct
{
	uint8_t id;
	float angle;   // 位置
	float speed;   // 速度
	float torque;  // 扭矩
	float temp;	   // 温度
	float errCode; // 故障码
} Motor_AK_RxData_t;

float Uint_To_Float(int x_int, float x_min, float x_max, int bits);
void Motor_AK_MIT_Decode(Motor_AK_RxData_t *rxData, uint8_t data[8], float pMax, float vMax, float tMax);

#endif
