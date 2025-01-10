/*
 * motor.c
 *
 *  Created on: Jan 9, 2025
 *      Author: krzysztof.tomicki
 */

#include "motor.h"

void Motor_Init(void)
{
	//set all control pins to low
	HAL_GPIO_WritePin(MOTOR_DIR_RIGHT_GPIO_Port, MOTOR_DIR_RIGHT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_DIR_LEFT_GPIO_Port, MOTOR_DIR_LEFT_Pin, GPIO_PIN_RESET);

	HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_RIGHT);
	HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_LEFT);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_RIGHT, 0);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_LEFT, 0);
}

void Motor_Stop(void)
{
	HAL_GPIO_WritePin(MOTOR_DIR_RIGHT_GPIO_Port, MOTOR_DIR_RIGHT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_DIR_LEFT_GPIO_Port, MOTOR_DIR_LEFT_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_RIGHT, 0);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_LEFT, 0);
}

void Motor_Percent_Control(const MotorDirection_t direction, int8_t velocity)
{
	//speed limits
	if (velocity > 100)
	{
		velocity = 100;
	}
	else if (velocity <= 0)
	{
		Motor_Stop();
		return;
	}

	//velocity percent into pulse conversion
	uint32_t pulse_value = (velocity * __HAL_TIM_GET_AUTORELOAD(&htim1))/100; //here pulse value will be set between 0 - 999

	//setting direction and PWM into STM pins
	if (direction == MOTOR_RIGHT)
	{
		//direction
		HAL_GPIO_WritePin(MOTOR_DIR_RIGHT_GPIO_Port, MOTOR_DIR_RIGHT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_DIR_LEFT_GPIO_Port, MOTOR_DIR_LEFT_Pin, GPIO_PIN_RESET);

		//PWM
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_RIGHT, pulse_value);
		HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_RIGHT);

		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_LEFT, 0);
		HAL_TIM_PWM_Stop(&htim1, MOTOR_PWM_LEFT);
	}
	else if (direction == MOTOR_LEFT)
	{
		//direction
		HAL_GPIO_WritePin(MOTOR_DIR_RIGHT_GPIO_Port, MOTOR_DIR_RIGHT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_DIR_LEFT_GPIO_Port, MOTOR_DIR_LEFT_Pin, GPIO_PIN_SET);

		//PWM
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_RIGHT, pulse_value);
		HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_LEFT);

		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_LEFT, 0);
		HAL_TIM_PWM_Stop(&htim1, MOTOR_PWM_RIGHT);
	}
}



