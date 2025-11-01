/*
 * 003_external_button_traffic_light.c
 *
 *  Created on: Oct 27, 2025
 *      Author: bryce
 */


#include "stm32f407xx.h"
void delay(void)
{
	for(uint32_t i = 0 ; i < 250000 ; i ++);
}

int main(void)
{
	GPIO_Handle_t GpioBtn;
	GPIO_Handle_t GpioGrn;
	GPIO_Handle_t GpioOrg;
	GPIO_Handle_t GpioRed;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GpioGrn.pGPIOx = GPIOD;
	GpioGrn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioGrn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioGrn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioGrn.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioGrn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioGrn);

	GpioOrg.pGPIOx = GPIOD;
	GpioOrg.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioOrg.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioOrg.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioOrg.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioOrg.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioOrg);

	GpioRed.pGPIOx = GPIOD;
	GpioRed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioRed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioRed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioRed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioRed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioRed);


	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioBtn);

	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, ENABLE);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, DISABLE);
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, DISABLE);

	uint8_t state = 1;
	while(1)
	{
	    if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == 0)
	    {
	        delay();
	        if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == 0)
	        {
	            switch(state)
	            {
	            case 0:
	                GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, DISABLE);
	                GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, ENABLE);
	                state = 1;
	                break;

	            case 1:
	                GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, DISABLE);
	                GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, ENABLE);
	                state = 2;
	                break;

	            case 2:
	                GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_13, DISABLE);
	                GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_14, ENABLE);
	                state = 0;
	                break;
	            }
	        }
	    }
	}
	return 0;

}

