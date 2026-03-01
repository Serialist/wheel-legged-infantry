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

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "user_lib.h"

/* General physics and mathematics constants ---------------------------------*/

/**
 * @brief the value of local gravity acceleration
 */

#define VAL_LIMIT(x, min, max) \
  do                           \
  {                            \
    if ((x) > (max))           \
    {                          \
      (x) = (max);             \
    }                          \
    else if ((x) < (min))      \
    {                          \
      (x) = (min);             \
    }                          \
  } while (0U)

#define GravityAccel 9.718f

/**
 * @brief Euler's Number
 */
#define Euler_Number 2.718281828459045f

/* Vision reslove constants -------------------------------------------------*/

// unit: degree
#define PITCH_MIN_ANGLE -20.0f
#define PITCH_MAX_ANGLE 20.0f

/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define Yaw_Distance_Decision 0

/**
 * @brief ballistic coefficient
 * @note  17mm: 0.038
 *        42mm: 0.019
 */
#define Bullet_Coefficient 0.038f

/**
 * @brief the half width of little armor
 */
#define LittleArmor_HalfWidth 0.07f

/**
 * @brief the half width of Large armor
 */
#define LargeArmor_HalfWidth 0.1175f

/* IMU reslove constants ---------------------------------------------------*/
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

/* Remote reslove constants -----------------------------------------------*/
/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN 0U

#endif // ROBOT_CONFIG_H
