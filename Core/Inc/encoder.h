/*
 * encoder.h
 *
 *  Created on: Jan 10, 2025
 *      Author: krzysztof.tomicki
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

//access to HAL
#include "stm32l4xx_hal.h"
#include "main.h"

typedef enum {
	ENCODER_5V,
	ENCODER_12V
} Encoder_Type_t;

void Encoder_Init(Encoder_Type_t encoder); //first function to call when using this library
uint16_t Encoder_GetRPM(Encoder_Type_t encoder);

#endif /* INC_ENCODER_H_ */
