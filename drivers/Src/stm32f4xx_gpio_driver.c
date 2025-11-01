/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Oct 16, 2025
 *      Author: bryce
 */

#include "stm32f4xx_gpio_driver.h"
/*
 * Peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) {
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    // 1. Configure mode
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        // interrupt mode config (if needed)
    	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
    	{
    		//1.configure the ftsr
    		EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


    	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
    	{
    		//1. configure the rtsr
    		EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
    	{
    		//1.configure the rtsr and ftsr
    		EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    	}
    	// 2. configure the gpio port selection in SYSCFG_EXTICR
    	uint8_t whichConfigRegister = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
    	uint8_t whichEXTI = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
    	// Clear out set port
    	SYSCFG->EXTICR[whichConfigRegister] &= ~( 0xF << whichEXTI);
    	// set new port
    	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
    	SYSCFG->EXTICR[whichConfigRegister] |= (portcode  << whichEXTI * 4);

    	// 3. enable the exti interrupt delivery using interrupt mask register
    	EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    temp = 0;
    // 2. Configure speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;
    // 3. Configure pull-up/pull-down
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;
    // 4. Configure output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;
    // 5. Configure alternate functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        uint8_t pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[reg] &= ~(0xF << (4 * pos));
        pGPIOHandle->pGPIOx->AFR[reg] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pos));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGIOx)
{
	uint16_t value;
	value = (uint16_t) pGIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1
		pGIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGIOx, uint16_t Value)
{
	pGIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber)
{
	pGIOx->ODR ^= (1 << PinNumber);
}
/*
 * IRC Configuration and handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	// Only need to use up to ISER2 since there are only 82 IRQ numbers supported by stm32F4.
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//ISER0
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if (IRQNumber < 64)
		{
			//ISER1
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

		}else if (IRQNumber < 96)
		{
			//ISER2
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );

		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			//ISER0
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if (IRQNumber < 64)
		{
			//ISER1
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

		}else if (IRQNumber < 96)
		{
			//ISER2
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );

		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t whichIprRegister = IRQNumber / 4;
	uint8_t whichIprSection= IRQNumber % 4;
	uint8_t shiftAmt = (8 * whichIprSection) + ( 8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + whichIprRegister) |= (IRQPriority << shiftAmt);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= ~(1 << PinNumber);
	}
}


