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

#define MOTOR_PWM_RIGHT TIM_CHANNEL_4
#define MOTOR_PWM_LEFT TIM_CHANNEL_3

//motor direction options
typedef enum {
	MOTOR_LEFT = 1,
	MOTOR_RIGHT = 0
} MotorDirection_t;

void Motor_Init(void); //first function to call when using this library
void Motor_Percent_Control(const MotorDirection_t direction, const uint8_t velocity); //sets motor in given direction and speed in range from 0 - 100

#endif /* INC_MOTOR_H_ */
