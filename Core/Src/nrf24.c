/*
 * nrf24.c
 *
 *  Created on: Oct 2, 2022
 *      Author: bonusoid
 */

#include "main.h"
#include "nrf24.h"
#include "nrf24_stm32f.h"

void NRF24_init()	//NRF24 Initialization
{
	NRF24_gpio_init();	//Init pin for CSN and CE

	NRF24_chip_disable();
	NRF24_reset(NRF24_REG_ALL);	//Reset all Register
	NRF24_write_reg(NRF24_CONFIG, 0);	//Default Configuration (CRC disabled)
	NRF24_write_reg(NRF24_EN_AA, 0);	//Auto Acknowledgment disabled
	NRF24_write_reg(NRF24_EN_RXADDR, 0);	//Disable all Data Pipe
	NRF24_write_reg(NRF24_SETUP_AW, 0x03);	//Use 5 bytes Address
	NRF24_write_reg(NRF24_SETUP_RETR, 0);	//Auto Retransmission disabled
	NRF24_write_reg(NRF24_RF_CH, 0);		//Set RF Channel to Channel 0 (reconfigured later)
	NRF24_write_reg(NRF24_RF_SETUP, 0x0E);	//Data Rate : 2 Mbps, TX Power : 0 dBm, LNA : low current
	NRF24_chip_enable();
}

void NRF24_reset(uint8_t regrst)	//Reset a Register or all Register
{
	/* NRF24 Registers Default Value (REG 0x00 - 0x17)*/
	uint8_t reg_def[] = {0x08,0x3F,0x03,0x03,0x03,0x02,0x0E,0x00,0x00,0x00,  //REG : 0x00 - 0x09
						 0x00,0x00,0xC3,0xC4,0xC5,0xC6,0x00,				 //REG : 0x0A - 0x10
						 	 	 	 	 	 	 	 	 	 	 	 	 	 //REG 0x0A,0x0B,0x10 : 5 byte
						 0x00,0x00,0x00,0x00,0x00,0x00,0x11};	    		 //REG : 0x11 - 0x17

	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};	//Default values for REG 0x0A (RX_ADDR_P0)
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};	//Default values for REG 0x0B (RX_ADDR_P1)
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};	//Default values for REG 0x10 (TX_ADDR)

	uint8_t regidx;

	if((regrst!=NRF24_RX_ADDR_P0) && (regrst!=NRF24_RX_ADDR_P1) && (regrst!=NRF24_TX_ADDR) && (regrst!=NRF24_DYNPD) && (regrst!=NRF24_FEATURE))
	{
		//Reset a Register between REG 0x00 - 0x17 (except 0x0A, 0x0B, and 0x10)
		NRF24_write_reg(regrst, reg_def[regrst]);
	}

	else if(regrst == NRF24_RX_ADDR_P0)	//Reset for REG 0x0A (RX_ADDR_P0)
	{
		NRF24_write_regN(NRF24_RX_ADDR_P0, rx_addr_p0_def, 5);	//Reset 5 bytes of RX Pipe 0 Address
	}

	else if(regrst == NRF24_RX_ADDR_P1)	//Reset for REG 0x0B (RX_ADDR_P1)
	{
		NRF24_write_regN(NRF24_RX_ADDR_P1, rx_addr_p1_def, 5);	//Reset 5 bytes of RX Pipe 1 Address
	}

	else if(regrst == NRF24_TX_ADDR)	//Reset for REG 0x10 (TX_ADDR)
	{
		NRF24_write_regN(NRF24_TX_ADDR, tx_addr_def, 5);	//Reset 5 bytes of TX Address
	}

	else if((regrst==NRF24_DYNPD) || (regrst==NRF24_FEATURE))	//Reset for REG 0x1C and 0x1D
	{
		NRF24_write_reg(regrst, 0x00);
	}

	else	//Reset for all Register
	{
		for(regidx=0;regidx<=0x09;regidx++)	//Reset for REG 0x00 - 0x09
		{
			NRF24_write_reg(regidx, reg_def[regidx]);
		}

		NRF24_write_regN(NRF24_RX_ADDR_P0, rx_addr_p0_def, 5);	//Reset for REG 0x0A
		NRF24_write_regN(NRF24_RX_ADDR_P1, rx_addr_p1_def, 5);	//Reset for REG 0x0B

		for(regidx=0x0C;regidx<=0x0F;regidx++)	//Reset for REG 0x0C - 0x0F
		{
			NRF24_write_reg(regidx, reg_def[regidx]);
		}

		NRF24_write_regN(NRF24_TX_ADDR, tx_addr_def, 5);	//Reset for REG 0x10

		for(regidx=0x11;regidx<=0x17;regidx++)	//Reset for REG 0x11 - 0x17
		{
			NRF24_write_reg(regidx, reg_def[regidx]);
		}

		NRF24_write_reg(NRF24_DYNPD, 0);	//Reset for REG 0x1C
		NRF24_write_reg(NRF24_FEATURE, 0);	//Reset for REG 0x1D
	}
}


void NRF24_send_cmd(uint8_t cmd)	//Send a Command
{
	NRF24_chip_select();

	NRF24_SPI_send(cmd);	//Send a Single byte Command via SPI

	NRF24_chip_deselect();
}

void NRF24_write_reg(uint8_t reg, uint8_t dat)	//Write Single byte to a Register
{
	uint8_t regbuf[2];
	regbuf[0] = reg|0x20;	//001AAAAA -> Write Register identifier
	regbuf[1] = dat;		//Data for Register

	NRF24_chip_select();

	NRF24_SPI_sendN(regbuf, 2);	//Send Register and Data via SPI

	NRF24_chip_deselect();
}

void NRF24_write_regN(uint8_t reg, uint8_t *Mdat, uint8_t Ndat)	//Write Multiple byte to a Register
{
	uint8_t regbuf;
	regbuf = reg|0x20;	//001AAAAA -> Write Register identifier

	NRF24_chip_select();

	NRF24_SPI_send(regbuf);	//Send Register via SPI
	NRF24_SPI_sendN(Mdat, Ndat); //Send Multiple Data of Register via SPI

	NRF24_chip_deselect();
}

uint8_t NRF24_read_reg(uint8_t reg)	//Read Single byte from a Register
{
	uint8_t dat = 0;

	NRF24_chip_select();

	NRF24_SPI_send(reg); //Send Register via SPI
	dat = NRF24_SPI_recv(); //Receive Single byte Data of Register via SPI

	NRF24_chip_deselect();

	return dat;
}

void NRF24_read_regN(uint8_t reg, uint8_t *Mdat, uint8_t Ndat)	//Read Multiple byte from a Register
{
	NRF24_chip_select();

	NRF24_SPI_send(reg); //Send Register via SPI
	NRF24_SPI_recvN(Mdat, Ndat); //Receive Multiple byte Data of Register via SPI

	NRF24_chip_deselect();
}


void NRF24_TXmode(uint8_t ch, uint8_t *addr)	//Set as Transmitter
{
	uint8_t config;

	NRF24_chip_disable();

	NRF24_write_reg(NRF24_RF_CH, ch);	//Select Channel
	NRF24_write_regN(NRF24_TX_ADDR, addr, 5);	//Write 5 bytes of TX Address

	//Set TX Configuration : Enable RF Power and Disable Primary RX
	config = NRF24_read_reg(NRF24_CONFIG);
	config = (config | 0x02) & 0xFE; //PWR_UP = 1; PRIM_RX = 0
	NRF24_write_reg(NRF24_CONFIG, config);

	NRF24_chip_enable();
}

uint8_t NRF24_TX(uint8_t *Mdat, uint8_t datlen)	//Transmit some byte of Data
{
	uint8_t cmd,fifost;

	NRF24_chip_select();

	//Send TX Payload Data
	cmd = NRF24_W_TX_PAYLOAD;
	NRF24_SPI_send(cmd);	//Command for Write Payload
	NRF24_SPI_sendN(Mdat, datlen); 	//Data size = datlen

	NRF24_chip_deselect();

	HAL_Delay(1);

	fifost = NRF24_read_reg(NRF24_FIFO_STATUS);

	if((fifost&(1<<4))&&(!(fifost&(1<<3)))) //Check if TX FIFO is empty
	{
		cmd = NRF24_FLUSH_TX;
		NRF24_send_cmd(cmd);	//Flush TX FIFO after used
		NRF24_reset(NRF24_FIFO_STATUS);	//Reset REG FIFO_STATUS

		return 1;
	}

	return 0;
}

void NRF24_RXmode(uint8_t ch, uint8_t npipe, uint8_t *addr, uint8_t ploadlen)	//Set as Receiver
{
	uint8_t en_rxaddr, config;
	uint8_t pipeNaddr[6] = {NRF24_RX_ADDR_P0,NRF24_RX_ADDR_P1,NRF24_RX_ADDR_P2,NRF24_RX_ADDR_P3,NRF24_RX_ADDR_P4,NRF24_RX_ADDR_P5};	//Pipe N Address
	uint8_t pipeNpload[6] = {NRF24_RX_PW_P0,NRF24_RX_PW_P1,NRF24_RX_PW_P2,NRF24_RX_PW_P3,NRF24_RX_PW_P4,NRF24_RX_PW_P5};	//Pipe N Payload Data size

	NRF24_chip_disable();

	NRF24_reset(NRF24_STATUS); //Reset REG STATUS
	NRF24_write_reg(NRF24_RF_CH, ch); //Select Channel

	//Select Data Pipe
	en_rxaddr = NRF24_read_reg(NRF24_EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<npipe); //select Data Pipe N
	NRF24_write_reg(NRF24_EN_RXADDR, en_rxaddr);

	//Write Data Pipe Address
	if(npipe<2) //for Pipe 0 and 1
		{
			NRF24_write_regN(pipeNaddr[npipe], addr, 5); //write Pipe N address
		}
	else 		//for Pipe 2-5
		{
			NRF24_write_regN(NRF24_RX_ADDR_P1, addr, 5); //first, write Pipe 1 address
			NRF24_write_reg(pipeNaddr[npipe], addr[0]); //then, write Pipe N address LSB
		}

	//Set Pipe Payload size/length
	NRF24_write_reg(pipeNpload[npipe], ploadlen); //Pipe N Payload size (max 32 bit)

	//Set TX Configuration : Enable RF Power and Primary RX
	config = NRF24_read_reg(NRF24_CONFIG);
	config = config | 0x03; //PWR_UP = 1; PRIM_RX = 1
	NRF24_write_reg(NRF24_CONFIG, config);

	NRF24_chip_enable();
}

uint8_t NRF24_checkpipe(uint8_t npipe)	//Check if Data is available in any Pipe
{
	uint8_t status;
	status = NRF24_read_reg(NRF24_STATUS);

	//Check RX_DR -> Check if Data ready in RX FIFO
	//Check if Data Pipe is ready
	if((status&(1<<6))&&(status&(npipe<<1)))
	{
		NRF24_write_reg(NRF24_STATUS, (1<<6));	//Write 1 to clear RX_DR
		return 1;
	}

	return 0;
}

void NRF24_RX(uint8_t *Mdat, uint8_t datlen)	//Receive some byte of Data
{
	uint8_t cmd;

	NRF24_chip_select();

	//Receive TX Payload Data
	cmd = NRF24_R_RX_PAYLOAD;
	NRF24_SPI_send(cmd);	//Command for Read Payload
	NRF24_SPI_recvN(Mdat, datlen); 	//Data size = datlen

	NRF24_chip_deselect();

	HAL_Delay(1);

	cmd = NRF24_FLUSH_RX;
	NRF24_send_cmd(cmd);	//Flush RX FIFO after used
}
