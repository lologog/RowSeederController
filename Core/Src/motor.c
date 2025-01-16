/*
 * motor.c
 *
 *  Created on: Jan 9, 2025
 *      Author: krzysztof.tomicki
 */

#include "motor.h"

//PID variables scaled by 1000 to send easier via CAN bus
static int32_t Kp = 0.8 * 1000;
static int32_t Ki = 0.5 * 1000;
static int32_t Kd = 0.1 * 1000;
static int32_t integral = 0;
static int32_t previous_error = 0;

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

void Motor_PID_Control(MotorDirection_t direction, int32_t desired_RPM, int32_t current_RPM)
{
	// P
    int32_t error = (int32_t)(desired_RPM - current_RPM);
    int32_t P = Kp * error / 1000;

    // I
    integral += error;

    //anti-windup limit
    if (integral > 100000) integral = 100000;
    if (integral < -100000) integral = -100000;

    int32_t I = Ki * integral / 1000;

    // D
    int32_t derivative = error - previous_error;
    int32_t D = Kd * derivative / 1000;

    //SUM
    int32_t output = P + I + D;

    //limit output range from 0 - 100% duty
    if (output > 100000) { output = 100000;}
    if (output < 0) { output = 0;}

    // making pwm from output value
    uint8_t pwm_duty_cycle = (uint8_t)(output / 1000);

    //Motor control
    Motor_Percent_Control(direction, pwm_duty_cycle);

    //update last error for next iteration
    previous_error = error;
}

