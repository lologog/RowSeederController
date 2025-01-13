/*
 * can.h
 *
 *  Created on: Jan 13, 2025
 *      Author: krzysztof.tomicki
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

//access to HAL
#include "stm32l4xx_hal.h"
#include "main.h"

void CAN_Init(CAN_HandleTypeDef *hcan);
int8_t CAN_SendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length);
void CAN_ConfigFilter(CAN_HandleTypeDef *hcan, uint32_t filter_id, uint32_t filter_mask);

#endif /* INC_CAN_H_ */
