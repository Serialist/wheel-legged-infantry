/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Config.c
 * @brief          : Configuare the Robot Functions
 * @author         : Yan Yuanbin
 * @date           : 2023/05/21
 * @version        : v1.0
 ******************************************************************************
 * @attention      : To be perfected
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "main.h"

/* ================ mode switch ================ */

#define BOARD_DM_MC02

// #define ZERO_FORCE

/// @brief 0: referee-system 1: image-trans
#define USART1_RX_Switch 0

/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN 0U

#define POWER_RESTRICT_ENABLE

/* ================ CAN ID ================ */

// 关节
#define HIP_LF_ID 0x03
#define HIP_LB_ID 0x04
#define HIP_RF_ID 0x02
#define HIP_RB_ID 0x01
// 轮毂
#define HUB_L_ID 0x202
#define HUB_R_ID 0x201
// 板间通信
#define B2B_CHASSIS_CMD_ID 0x199
#define YAW_ID 0x203

/* ================ environment parameter ================ */

#define GRAVITY_ACCEL 9.718f

#define GravityAccel GRAVITY_ACCEL

/* ================ robot parameter ================ */

#define WHEEL_MASS 0.513 // kg
#define ROD1_MASS 1      // kg
#define ROD2_MASS 13.4   // kg

#define WHEEL_INERTIA 0.000956   // kg m^2
#define ROD1_INERTIA 0           // kg m^2
#define ROD2_INERTIA 0.353588749 // kg m^2

#define WHEEL_RADIUS 0.068 // m
#define ROD1_LEN 0         // m
#define ROD2_LEN 0.036     // m

#define BODY_WIDTH 0.54 // m

#define YAW_ENCODER_OFFSET 7777 // rmmotor encoder

#define MOTOR_PARAM_K1 0.805767
#define MOTOR_PARAM_K2 1.049220
#define MOTOR_PARAM_K3 -1.361828

/* ================ ins ================ */

/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE 0U

#define IMU_ANGLE_INDEX_PITCH 1U
#define IMU_ANGLE_INDEX_YAW 0U
#define IMU_ANGLE_INDEX_ROLL 2U

#define IMU_GYRO_INDEX_PITCH 1U
#define IMU_GYRO_INDEX_YAW 2U
#define IMU_GYRO_INDEX_ROLL 0U

#define IMU_ACCEL_INDEX_PITCH 0U
#define IMU_ACCEL_INDEX_YAW 2U
#define IMU_ACCEL_INDEX_ROLL 1U

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_H
