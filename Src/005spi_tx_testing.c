/*
 * 005spi_tx_testing.c
 *
 *  Created on: Nov 9, 2025
 *      Author: bryce
 */

#include <string.h>
#include "stm32f407xx.h"

//TODO
/*
 * Send Hellow World and use the following configs:
 * SPI-2 Master Mode
 * SCLK max possible
 * DFF 0 and DFF 1
 *
 * Note: Don't need to actually connect a slave.
 *
 * Config: yes MOSI, not MISO, not NSS, and yes SCLK. 2 Pins.
 * MOSI: PB15
 * MISO: PB14
 * SCLK: PB13
 * NSS:	 PB12
 * Alt function mode: 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB; //PB so GPIOB
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //enable for nss pin since we arent using the pin itself.

	SPI_Init(&SPI2Handle);
}

int main(void)
{

	char user_data[] = "Hello World";

	//Init the gpio pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// Enable SSI since we are using SSM to pull NSS to high
	SPI_SSIConfig(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	while(1)
	{
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));
	    for (volatile int i = 0; i < 100000; i++); // short delay
	}

	return 0;
}



