/*
 * encoder.c
 *
 *  Created on: Jan 10, 2025
 *      Author: krzysztof.tomicki
 */

#include "encoder.h"

#define IMPULSES_PER_ROTATION (462)
#define SECONDS_IN_MINUTE (60)
#define PE_PER_IMPULSE (4)

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

uint32_t Encoder_GetScaledRPM(Encoder_Type_t encoder)
{
	uint32_t impulses = 0; //store value of impulses for one iteration and reset this value on every entry
	uint32_t RPM_scaled = 0; //rotates per minute but scaled *1000 to avoid using floats
	uint16_t scale = 1000;

	impulses = htim4.Instance->CNT; //get number of positive edges detected by timer connected to encoder
	impulses /= PE_PER_IMPULSE; //4 positive edges per one impulse on encoder

	RPM_scaled = (impulses * scale * SECONDS_IN_MINUTE) / IMPULSES_PER_ROTATION;

	__HAL_TIM_SET_COUNTER(&htim4, 0); //reset counter to 0
	return RPM_scaled;
}

/* EXAMPLE HOW TO USE THIS CODE
 *
 *		Encoder_Init(ENCODER_5V);
 *
 *     	uint32_t RPM_scaled = Encoder_GetScaledRPM(ENCODER_5V);

    	//We need to split 32 bit int into 4x8bit bytes to send via CAN bus
    	uint8_t byte1 = (RPM_scaled >> 24) & 0xFF; //MSB
    	uint8_t byte2 = (RPM_scaled >> 16) & 0xFF;
    	uint8_t byte3 = (RPM_scaled >> 8) & 0xFF;
    	uint8_t byte4 = RPM_scaled & 0xFF; //LSB

    	CAN_SendMessage(&hcan1, 0x200, (uint8_t[]){byte1, byte2, byte3, byte4}, 4);
 *
 */
