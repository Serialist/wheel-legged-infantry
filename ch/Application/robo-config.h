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
extern "C"
{
#endif

#define BOARD_DM_MC02

#define ZERO_FORCE

// CAN ID
// 关节
#define HIP_LF_ID 0x03
#define HIP_LB_ID 0x04
#define HIP_RF_ID 0x01
#define HIP_RB_ID 0x02
// 轮毂
#define HUB_L_ID 0x201
#define HUB_R_ID 0x202
// 板间通信
#define B2B_CHASSIS_CMD_ID 0x666

/// @brief 0: referee-system 1: image-trans
#define USART1_RX_Switch 0

#define GravityAccel 9.718f

#define WHEEL_RADIUS 0.066f

/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE 0U

#define IMU_ANGLE_INDEX_PITCH 2U
#define IMU_ANGLE_INDEX_YAW 0U
#define IMU_ANGLE_INDEX_ROLL 1U

#define IMU_GYRO_INDEX_PITCH 0U
#define IMU_GYRO_INDEX_YAW 2U
#define IMU_GYRO_INDEX_ROLL 1U

#define IMU_ACCEL_INDEX_PITCH 0U
#define IMU_ACCEL_INDEX_YAW 2U
#define IMU_ACCEL_INDEX_ROLL 1U

/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN 0U

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONFIG_H
