/*
 * nrf24_stm32f.h
 *
 *  Created on: Oct 2, 2022
 *      Author: bonusoid
 */

#ifndef INC_NRF24_STM32F_H_
#define INC_NRF24_STM32F_H_

/* NRF24 SPI */
extern SPI_HandleTypeDef hspi1;	// SPI1 definition
#define NRF24_SPI	hspi1

/* NRF24 GPIO */
#define NRF24_CSN	GPIO_PIN_0	// Chip Select (active low)
#define NRF24_CE	GPIO_PIN_1	// Chip Enable (active high)
#define NRF24CTRL	GPIOF		// CSN and CE Port
void NRF24_gpio_init();			// Init Function for CSN and CE

/* NRF24 Control Function */
void NRF24_chip_enable();	//NRF24-CE = 1
void NRF24_chip_disable();	//NRF24-CE = 0
void NRF24_chip_select();	//NRF24-CSN = 0
void NRF24_chip_deselect();	//NRF24-CSN = 1

/* NRF24 SPI Access */
void NRF24_SPI_send(uint8_t dat); //Send Single byte of SPI Data
void NRF24_SPI_sendN(uint8_t *Mdat, uint8_t Ndat); //Send Multiple byte of SPI Data
uint8_t NRF24_SPI_recv(); //Receive Single byte of SPI Data
void NRF24_SPI_recvN(uint8_t *Mdat, uint8_t Ndat); //Receive Multiple byte of SPI Data

#endif /* INC_NRF24_STM32F_H_ */
