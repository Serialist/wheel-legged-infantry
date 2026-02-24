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

typedef struct
{

  enum
  {
    SHOOT_NONE = 0,
    SHOOT_READY,    // 初始化完成
    SHOOT_START,    // 开摩擦轮
    SHOOT_SEMIAUTO, // 半自动（单发，无热量控制）
    SHOOT_FULLAUTO, // 全自动（连发，有热量控制）
    SHOOT_UNLOAD,   // 卸弹
  } state;

  struct
  {

    float farc1;
    float farc2;
    float feed_freq;
    float feed_num;

  } target;

  struct
  {

    float farc1;
    float farc2;
    float feed_angle;

  } feedback;

  struct
  {

    int16_t farc1;
    int16_t farc2;
    int16_t ammo_feed;

  } output;

} Shoot_Info_Typedef;

extern Shoot_Info_Typedef shoot_info;

#endif // SHOOT_H