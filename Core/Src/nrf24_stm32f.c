/*
 * nrf24_stm32.c
 *
 *  Created on: Oct 2, 2022
 *      Author: bonusoid
 */

#include "main.h"
#include "nrf24_stm32f.h"

void NRF24_gpio_init() // Init Function for CSN and CE
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF24CTRL, NRF24_CSN|NRF24_CE, GPIO_PIN_RESET);

	/*Configure GPIO pins : PF0 PF1 */
	GPIO_InitStruct.Pin = NRF24_CSN|NRF24_CE;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NRF24CTRL, &GPIO_InitStruct);
}


void NRF24_chip_enable()	//NRF24-CE = 1
{
	HAL_GPIO_WritePin(NRF24CTRL, NRF24_CE, GPIO_PIN_SET);
}

void NRF24_chip_disable()	//NRF24-CE = 0
{
	HAL_GPIO_WritePin(NRF24CTRL, NRF24_CE, GPIO_PIN_RESET);
}

void NRF24_chip_select()	//NRF24-CSN = 0
{
	HAL_GPIO_WritePin(NRF24CTRL, NRF24_CSN, GPIO_PIN_RESET);
}

void NRF24_chip_deselect()	//NRF24-CSN = 1
{
	HAL_GPIO_WritePin(NRF24CTRL, NRF24_CSN, GPIO_PIN_SET);
}


void NRF24_SPI_send(uint8_t dat)	//Send Single byte of SPI Data
{
	HAL_SPI_Transmit(&NRF24_SPI, &dat, 1, 100);
}

void NRF24_SPI_sendN(uint8_t *Mdat, uint8_t Ndat)	//Send Multiple byte of SPI Data
{
	HAL_SPI_Transmit(&NRF24_SPI, Mdat, Ndat, 1000);
}

uint8_t NRF24_SPI_recv()	//Receive Single byte of SPI Data
{
	uint8_t dat = 0;

	HAL_SPI_Receive(&NRF24_SPI, &dat, 1, 100);

	return dat;
}

void NRF24_SPI_recvN(uint8_t *Mdat, uint8_t Ndat)	//Receive Multiple byte of SPI Data
{
	HAL_SPI_Receive(&NRF24_SPI, Mdat, Ndat, 1000);
}

