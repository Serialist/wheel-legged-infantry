/************************
 * @file debug.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-11-22
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "debug.h"
#include "user_lib.h"
#include <math.h>

#include "tim.h"
#include "bsp_buzzer.h"

extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

// float Rampf(float x, float x0, float kmax, float kmin, float dt) :
// {
//     float dx = fClam (kmax * dt, x0 - x);

//     return new_channel;
// }
