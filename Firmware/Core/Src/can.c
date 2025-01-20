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
	5. to receive a message you have to use callback function - all deatail are written below

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
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
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

//filters what can go into FIFO based on ID and IDmask
void CAN_ConfigFilters(CAN_HandleTypeDef *hcan, uint32_t filter_id, uint32_t filter_mask) {
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;             // enable filtering
    canfilterconfig.FilterBank = 0;                                   // set filter bank
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;              // where should this filter work (FIFO0)
    canfilterconfig.FilterIdHigh = (filter_id << 5) & 0xFFFF;         // ID high
    canfilterconfig.FilterIdLow = 0x0000;                             // ID low
    canfilterconfig.FilterMaskIdHigh = (filter_mask << 5) & 0xFFFF;   // maskID high
    canfilterconfig.FilterMaskIdLow = 0x0000;                         // maskID low
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;               // filter mode here mask id
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;              // filter has 32 bits
    canfilterconfig.SlaveStartFilterBank = 14;                        // set slave bank, useless if using only one CAN

    if (HAL_CAN_ConfigFilter(hcan, &canfilterconfig) != HAL_OK)
    {
    	//error
        Error_Handler();
    }
}

/* To receive a message the best will be a callback function
 * below is shown an example function
 *
 *
//interrupt if something went through CAN filters and is in FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxData) == HAL_OK)
    {
    	//function with logic
        Process_CAN_Message(RxHeader.StdId, rxData, RxHeader.DLC);
    }
}

void Process_CAN_Message(uint32_t id, uint8_t *data, uint8_t length)
{
	uint8_t data_known[8] = {0x03, 0x03, 0x03};
	uint8_t data_unknown[8] = {0x06, 0x06, 0x06};
    if (id == 0x123)
    {
    	CAN_SendMessage(&hcan1, 0x103, data_known, 3);
    }
    else
    {
    	CAN_SendMessage(&hcan1, 0x103, data_unknown, 3);
    }
}
 */
