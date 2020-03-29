#include "cs5451.h"

void cs5451_reset(uint8_t on)
{
	HAL_GPIO_WritePin(ADC_Reset_GPIO_Port, ADC_Reset_Pin,  on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}
void cs5451_gain(uint8_t on)
{
	HAL_GPIO_WritePin(ADC_Gain_GPIO_Port, ADC_Gain_Pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}
void cs5451_owrs(uint8_t on)
{
	HAL_GPIO_WritePin(ADC_OWRS_GPIO_Port, ADC_OWRS_Pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}
void cs5451_se(uint8_t on)
{
	HAL_GPIO_WritePin(ADC_SE_GPIO_Port, ADC_SE_Pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}
