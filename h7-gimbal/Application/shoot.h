/**
 * @file shoot.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef SHOOT_H
#define SHOOT_H

#include "user_lib.h"

#define SHOOT_FEEDER_ROTATE_RATIO 1

typedef enum
{
  SHOOT_NONE = 0,
  SHOOT_READY,      // 初始化完成
  SHOOT_START,      // 开摩擦轮
  SHOOT_SEMIAUTO,   // 半自动（单发，无热量控制）
  SHOOT_FULLAUTO,   // 全自动（连发，有热量控制）
  SHOOT_SINGLE,     // 单发
  SHOOT_CONTINUOUS, // 连发
  SHOOT_UNLOAD,     // 退弹
} Shoot_Status_t;

typedef struct
{
  Shoot_Status_t status;

  struct
  {

    float fr;
    float feed_freq;
    float feed_num;

  } target;

  struct
  {

    float fr[2];
    float feed;

  } feedback;

  struct
  {

    int16_t fr[2];
    int16_t feed;

  } output;

} Shoot_t;

extern Shoot_t shoot;

#endif // SHOOT_H