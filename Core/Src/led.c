/*
 * led.c
 *
 *  Created on: Jan 9, 2025
 *      Author: krzysztof.tomicki
 */

#include "led.h"
#include "main.h"

void LED_On(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void LED_Off(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
