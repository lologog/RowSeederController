/*
 * led.c
 *
 *  Created on: Jan 9, 2025
 *      Author: krzysztof.tomicki
 */

#include "led.h"
#include "main.h"

#define DEFAULT_BLINK_INTERVAL 100

void LED_On(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void LED_Off(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void LED_Blink(int16_t interval_ms)
{
	static uint32_t last_toggle_time = 0; //time from last led state change

	//protection for negative or 0 user input
	if (interval_ms <= 0)
	{
		interval_ms = DEFAULT_BLINK_INTERVAL; //default value
	}

	//take current time
	uint32_t current_time = HAL_GetTick();

	//check if interval_ms is already passed
	if ((current_time - last_toggle_time) >= interval_ms)
	{
		//reset time
		last_toggle_time = current_time;

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}
