/*
 * motor.h
 *
 *  Created on: Jan 9, 2025
 *      Author: krzysztof.tomicki
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

//access to HAL
#include "stm32l4xx_hal.h"
#include "main.h"

#define MOTOR_PWM_RIGHT TIM_CHANNEL_4
#define MOTOR_PWM_LEFT TIM_CHANNEL_3

//motor direction options (global)
typedef enum {
	MOTOR_LEFT,
	MOTOR_RIGHT
} MotorDirection_t;

//frequency used in PWM is 10 kHz

void Motor_Init(void); //first function to call when using this library
void Motor_Percent_Control(const MotorDirection_t direction, const int8_t velocity); //sets motor in given direction and speed in range from 0 - 100
void Motor_Stop(void); //motor doesnt receive any control signals
void Motor_PID_Control(MotorDirection_t direction, int32_t desired_RPM, int32_t current_RPM); //PID regulation to desired rotational speed

#endif /* INC_MOTOR_H_ */
