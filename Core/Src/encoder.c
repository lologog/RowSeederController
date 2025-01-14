/*
 * encoder.c
 *
 *  Created on: Jan 10, 2025
 *      Author: krzysztof.tomicki
 */

#include "encoder.h"

#define IMPULSES_PER_ROTATION (462)
#define SECONDS_IN_MINUTE (60)
#define SCALE_FACTOR (1000)
#define PE_PER_IMPULSE (4)

/*	INSTRUCTIONS HOW TO USE THIS LIBRARY
 *	1. Turn on Timers in encoder mode
 * 	2. Use init functions on timers
 * 	3. You need to set one internal timer to make overfloat interrupt with f=1Hz
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

uint32_t Encoder_GetScaledRPM(Encoder_Type_t encoder)
{
	//choose encoder timer
    TIM_HandleTypeDef *htim;
    switch (encoder)
    {
        case ENCODER_5V:
            htim = &htim4;
            break;
        case ENCODER_12V:
            htim = &htim8;
            break;
        default:
            Error_Handler();
            return 0;
    }

    //calculations
    uint32_t impulses = htim->Instance->CNT;
    impulses /= PE_PER_IMPULSE;

    uint32_t RPM_scaled = (impulses * SCALE_FACTOR * SECONDS_IN_MINUTE) / IMPULSES_PER_ROTATION;

    __HAL_TIM_SET_COUNTER(htim, 0);
    return RPM_scaled;
}

