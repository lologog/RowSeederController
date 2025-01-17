/*
 * can_logic.c
 *
 *  Created on: Jan 17, 2025
 *      Author: krzysztof.tomicki
 */

#include "can_logic.h"
#include "can.h"
#include "led.h"

extern uint8_t RPM;

void Process_CAN_Message(uint32_t id, uint8_t *data, uint8_t length)
{
    switch (id)
    {
    case 0x01: //ping
    {
        uint8_t ping[1] = {1};
        CAN_SendMessage(&hcan1, 0x03, ping, 1);
        break;
    }

    case 0x02: //turn LED ON
        LED_On();
        break;

    case 0x03: //turn LED OFF
        LED_Off();
        break;

    case 0x04: //change state of LED
    	LED_Toggle();
    	break;

    case 0x05: //read Dist sensor analog measurement
    	Read_ADC(&hadc1, ADC_CHANNEL_6);
    	break;

    case 0x06:
    	Read_ADC(&hadc1, ADC_CHANNEL_7);
    	break;

    case 0x07: //turn on power switches
    	PowerSwitchOnOffDiagnostic(1, GPIO_PIN_SET);
    	break;

    case 0x08:
    	PowerSwitchOnOffDiagnostic(2, GPIO_PIN_SET);
    	break;

    case 0x09:
    	PowerSwitchOnOffDiagnostic(3, GPIO_PIN_SET);
    	break;

    case 0x10:
    	PowerSwitchOnOffDiagnostic(4, GPIO_PIN_SET);
    	break;

    case 0x11:
    	PowerSwitchOnOffDiagnostic(5, GPIO_PIN_SET);
    	break;

    case 0x12:
    	PowerSwitchOnOffDiagnostic(6, GPIO_PIN_SET);
    	break;

    case 0x13:
    	RPM = data[0];
    	break;

    default: //unknown id
        Error_Handler();
        break;
    }
}

