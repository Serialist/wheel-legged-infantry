/**
 * @file buzzer.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-18
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "buzzer.h"
#include "user_lib.h"

typedef const uint16_t BZ_Sound_t[][2];

BZ_Sound_t BZSOUND_1 = {{0x00, 0x01}, {0x02, 0x03}, {0x04, 0x05}, {0x06, 0x07}, {0x08, 0x09}};

void Buzzer_Init(void) {}

void Buzzer_Play(uint16_t freq, uint16_t time)
{
    // open
    // PWM_Set(PWM_CH_BUZZER, freq);
    // close
}

void Buzzer_Sound(BZ_Sound_t sound, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        Buzzer_Play(sound[i][0], sound[i][1]);
    }
}
