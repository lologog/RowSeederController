/*
 * encoder.c
 *
 *  Created on: Jan 10, 2025
 *      Author: krzysztof.tomicki
 */

#include "encoder.h"
#include "can.h"

/*	INSTRUCTIONS HOW TO USE THIS LIBRARY
 *	1. Turn on Timers in encoder mode
 * 	2. Use init functions on timers
 * 	3. You need to set one internal timer to make overfloat interrupt with f=1000Hz
 * 	4. Enable interrupts e.g. HAL_TIM_Base_Start_IT(&htim2);
 * 	5. Write function to handle interrupts void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
 * 	6. Use Encoder_GetRPM function inside Callbackfunction to get RPM
 */

void Encoder_Init(Encoder_Type_t encoder)
{
	// choose one of 2 encoders built in controller
	switch (encoder)
	{
	case ENCODER_5V:
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //start encoder
		__HAL_TIM_SET_COUNTER(&htim4, 0); //reset counter to 0
		break;
	case ENCODER_12V:
		HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		break;
	default:
		break;
	}
}

uint16_t Encoder_GetRPM(Encoder_Type_t encoder)
{
	uint16_t impulses = 0;

	impulses = htim4.Instance->CNT;
	impulses /= 4; //4 positive edges per one impulse on encoder
	__HAL_TIM_SET_COUNTER(&htim4, 0); //reset counter to 0
	return impulses;
}
