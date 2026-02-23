/**
 * @file buzzer.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-18
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef BUZZER_H
#define BUZZER_H

typedef const uint16_t BZ_Sound_t[][2];
extern BZ_Sound_t BZSOUND_1;

void Buzzer_Init(void);

void Buzzer_Play(uint16_t freq, uint16_t time);

void Buzzer_Sound(BZ_Sound_t sound, uint16_t len);

#endif
