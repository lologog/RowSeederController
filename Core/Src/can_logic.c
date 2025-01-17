/*
 * can_logic.c
 *
 *  Created on: Jan 17, 2025
 *      Author: krzysztof.tomicki
 */

#include "can_logic.h"
#include "can.h"

void Process_CAN_Message(uint32_t id, uint8_t *data, uint8_t length)
{
	if (id == 0x01)
	{
		uint8_t ping[8] = {1};
		CAN_SendMessage(&hcan1, 0x002, ping, 1);
	}
}

