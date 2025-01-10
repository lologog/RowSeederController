/*
 * encoder.c
 *
 *  Created on: Jan 10, 2025
 *      Author: krzysztof.tomicki
 */

#include "encoder.h"

void Encoder_Init(Encoder_t encoder)
{
	// choose one of 2 encoders built in controller
	switch (encoder)
	{
	case ENCODER_1:
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //start encoder
		__HAL_TIM_SET_COUNTER(&htim4, 0); //reset counter to 0
		break;
	case ENCODER_2:
		HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		break;
	default:
		break;
	}
}
