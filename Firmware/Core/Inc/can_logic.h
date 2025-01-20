/*
 * can_logic.h
 *
 *  Created on: Jan 17, 2025
 *      Author: krzysztof.tomicki
 */

#ifndef INC_CAN_LOGIC_H_
#define INC_CAN_LOGIC_H_

//access to HAL
#include "stm32l4xx_hal.h"
#include "main.h"

void Process_CAN_Message(uint32_t id, uint8_t *data, uint8_t length);

#endif /* INC_CAN_LOGIC_H_ */
