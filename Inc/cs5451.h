#pragma once

#include "stm32f1xx_hal.h"
#include "main.h"

void cs5451_reset(uint8_t on);
void cs5451_gain(uint8_t on);
void cs5451_owrs(uint8_t on);
void cs5451_se(uint8_t on);
