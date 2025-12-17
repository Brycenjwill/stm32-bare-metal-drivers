/*
 * stm32f4xx_I2C_driver.c
 *
 *  Created on: Dec 12, 2025
 *      Author: bryce
 */

#include "stm32f4xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

uint32_t RCC_GetPLLOutpoutClock(void)
{
	return 0;
}

uint16_t AHB_PreScalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScalar[8] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apbp1;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	} else if(clksrc == 1)
	{
		SystemClk = 8000000;
	} else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutpoutClock();
	}

	// AHB1
	temp = (RCC->CFGR  >> 4) & 0xF;

	if(temp < 8)
	{
		ahbp = 1;
	} else
	{
		ahbp = AHB_PreScalar[temp - 8];
	}

	// APB1
	temp = (RCC->CFGR  >> 10) & 0x7;

	if(temp < 4)
	{
		apbp1 = 1;
	} else
	{
		apbp1 = APB1_PreScalar[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apbp1;

	return pclk1;
}

/*
 * Peripheral clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) {
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// Steps (Done while peripheral is disabled in the CR)
	// 1. Configure the mode (std or fast)
	// 2. Configure the serial clock speed
	// 3. Configure the device address (Applicable when the device is slave)
	// 4. Enable the Acking (Disabled by default in stm32f4xx)
	// 5. Configure the clock control register (mode and ccr value)
	// 6. Configure the rise time for I2C pins

	uint32_t tempreg = 0;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	// ack control bit init (it doesn't automatically ack by default)
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// freq init
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F); // first 5 bits are freq

	// device address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// ccr (clock control register)
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard
		ccr_value |= RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	} else
	{
		// mode is fast
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard
		tempreg |= (RCC_GetPCLK1Value() / 1000000U) + 1;

	} else
	{
		// mode is fast
		tempreg |= ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_PCLK_DI();
	}
	else if (pI2Cx == I2C2)
	{
		I2C1_PCLK_DI();
	}
	else if (pI2Cx == I2C3)
	{
		I2C1_PCLK_DI();
	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_SPE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_SPE);
	}
}

void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t whichIprRegister = IRQNumber / 4;
	uint8_t whichIprSection= IRQNumber % 4;
	uint8_t shiftAmt = (8 * whichIprSection) + ( 8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + whichIprRegister) |= (IRQPriority << shiftAmt);
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ExAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress &= ~(1); /// Clear first bit for write
	pI2Cx->DR = SlaveAddress;
}

void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	// ADDR flag is cleared by reading sr1 and sr2
	uint32_t read = pI2Cx->SR1;
	read = pI2Cx->SR2;
	(void)read;
}


// Data send and recieve
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Check SB flag in SR1 to confirm start is set
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// Send the address of the slave with read / write bit set (8th bit)
	I2C_ExAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// Check ADDR flag to check that address phase is completed
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Clear the addr flag
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// Send data until len is 0
	while(Len > 0)
	{
		// What for tx to become empty
		while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer ++;
		Len --;
	}

	// Wait for TXE to be 1 and BTF to be 1
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

	// Generate the stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}





