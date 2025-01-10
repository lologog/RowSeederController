/*
 * led.h
 *
 *  Created on: Jan 9, 2025
 *      Author: krzysztof.tomicki
 */

#ifndef INC_LED_H_
#define INC_LED_H_

//access to HAL
#include "stm32l4xx_hal.h"
#include "main.h"

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Blink(int16_t interval_ms);

#endif /* INC_LED_H_ */
