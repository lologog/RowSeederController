/*
 * can.c
 *
 *  Created on: Jan 13, 2025
 *      Author: krzysztof.tomicki
 */

#include "can.h"

/*
	INSTRUCTION TO USE THIS LIBRARY:
	1. configure CAN in your main.c file (Baud Rate (I was testing this code on 500 000 bit/s) and make CANX RX0 interrupt enabled)
	2. turn on CAN e.g. CAN_Init(&hcan1);
	3. create an array e.g. uint8_t data_to_send[8] = {0x01, 0x02, 0x03};
	4. send a message by using CAN e.g. CAN_SendMessage(&hcan1, 0x103, data_to_send, 3);
*/

//private - struct is used in case we want to have more than one CAN bus
typedef struct {
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
    uint32_t TxMailbox;
} CAN_t;

//Initialisation of CAN bus
void CAN_Init(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);

	//can filter configuration
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0x0000;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
	{
		//error
		Error_Handler();
	}
}

// send data frame via CAN bus
int8_t CAN_SendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length)
{
	CAN_t can_t;

	//by value bcs we are working on can_t locally
	can_t.TxHeader.DLC = length; //number of bytes to send
	can_t.TxHeader.IDE = CAN_ID_STD;
	can_t.TxHeader.RTR = CAN_RTR_DATA;
	can_t.TxHeader.StdId = id;

	if (HAL_CAN_AddTxMessage(hcan, &can_t.TxHeader, data, &can_t.TxMailbox) != HAL_OK)
	{
		//error
		return -1;
	}
	return 0; //everything is fine
}
