/*
 * 004_external_button_interrupt.c
 *
 *  Created on: Nov 2, 2025
 *      Author: bryce
 */

#include "stm32f407xx.h"

int main(void)
{
	// TODO
	// Connect an external button to PD5. Toggle LD2 whenever interrupt is triggered
	// by button press. Use falling edge interrupt trigger.
	// Peripheral setup: Connet a button to PD5 and to a common ground.

	GPIO_Handle_t GpioBtn;
	GPIO_Handle_t GpioGrn;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	// Init green onboard led (D12)
	GpioGrn.pGPIOx = GPIOD;
	GpioGrn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioGrn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioGrn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioGrn.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioGrn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Init external button GPIO pin (B12)
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;



	GPIO_Init(&GpioBtn);
	GPIO_Init(&GpioGrn);


	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9, NVIC_IRQ_PRI15);
	GPIO_IRQITConfig(IRQ_NO_EXTI5_9, ENABLE);
	return 0;
}

// Overwrite ISR
void EXTI9_5_IRQHandler(void)
{
	// Tell the cpu that the interrupt has been handled
	GPIO_IRQHandling(GPIO_PIN_NO_5);

	// Toggle the led
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}


