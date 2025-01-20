/*
 * can_logic.c
 *
 *  Created on: Jan 17, 2025
 *      Author: krzysztof.tomicki
 */

#include "can_logic.h"
#include "can.h"
#include "led.h"
#include "motor.h"

extern uint16_t RPM;
extern MotorDirection_t direction;
extern uint8_t motor_percent_flag;
extern uint8_t duty;

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

    case 0x07: //turn on/off power switches
    	PowerSwitchOnOffDiagnostic(1, data[0] == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	break;

    case 0x08:
    	PowerSwitchOnOffDiagnostic(1, data[0] == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	break;

    case 0x09:
    	PowerSwitchOnOffDiagnostic(1, data[0] == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	break;

    case 0x10:
    	PowerSwitchOnOffDiagnostic(1, data[0] == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	break;

    case 0x11:
    	PowerSwitchOnOffDiagnostic(1, data[0] == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	break;

    case 0x12:
    	PowerSwitchOnOffDiagnostic(1, data[0] == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	break;

    case 0x13: //motor PID control
    	RPM = (data[1] << 8) | data[0];

    	uint8_t dir_temp = data[2];

    	if (dir_temp == 1)
    	{
    		direction = MOTOR_RIGHT;
    	}
    	else if (dir_temp == 2)
    	{
    		direction = MOTOR_LEFT;
    	}
    	else
    	{
    		Error_Handler();
    	}
    	break;

    case 0x14: //motor stop
    	RPM = 0;
    	direction = MOTOR_LEFT;
    	duty = 0;
    	motor_percent_flag = 0;
    	Motor_Stop();
    	break;

    case 0x15: //motor percent control
    	motor_percent_flag = 1;

    	duty = data[0];

    	if (duty > 100)
    	{
    		duty = 100;
    	}

    	uint8_t dir_temp_percent = data[1];

    	if (dir_temp_percent == 1)
    	{
    		direction = MOTOR_RIGHT;
    	}
    	else if (dir_temp_percent == 2)
    	{
    		direction = MOTOR_LEFT;
    	}
    	else
    	{
    		Error_Handler();
    	}
    	break;


    default: //unknown id
        Error_Handler();
        break;
    }
}

