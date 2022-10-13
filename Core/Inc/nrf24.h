/*
 * nrf24.h
 *
 *  Created on: Oct 2, 2022
 *      Author: bonusoid
 */

#ifndef INC_NRF24_H_
#define INC_NRF24_H_

/* NRF24 COMMAND */
#define NRF24_R_REGISTER    0x00
#define NRF24_W_REGISTER    0x20
#define NRF24_REGISTER_MASK 0x1F
#define NRF24_ACTIVATE      0x50
#define NRF24_R_RX_PL_WID   0x60
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_W_ACK_PAYLOAD 0xA8
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2
#define NRF24_REUSE_TX_PL   0xE3
#define NRF24_NOP           0xFF

/* NRF24 REGISTER */
#define NRF24_CONFIG      	0x00
#define NRF24_EN_AA       	0x01
#define NRF24_EN_RXADDR   	0x02
#define NRF24_SETUP_AW    	0x03
#define NRF24_SETUP_RETR  	0x04
#define NRF24_RF_CH       	0x05
#define NRF24_RF_SETUP    	0x06
#define NRF24_STATUS      	0x07
#define NRF24_OBSERVE_TX  	0x08
#define NRF24_CD          	0x09
#define NRF24_RX_ADDR_P0  	0x0A
#define NRF24_RX_ADDR_P1  	0x0B
#define NRF24_RX_ADDR_P2  	0x0C
#define NRF24_RX_ADDR_P3  	0x0D
#define NRF24_RX_ADDR_P4  	0x0E
#define NRF24_RX_ADDR_P5  	0x0F
#define NRF24_TX_ADDR     	0x10
#define NRF24_RX_PW_P0    	0x11
#define NRF24_RX_PW_P1    	0x12
#define NRF24_RX_PW_P2    	0x13
#define NRF24_RX_PW_P3    	0x14
#define NRF24_RX_PW_P4    	0x15
#define NRF24_RX_PW_P5    	0x16
#define NRF24_FIFO_STATUS 	0x17
#define NRF24_DYNPD	    	0x1C
#define NRF24_FEATURE	    0x1D

#define NRF24_REG_ALL		0x20	// Identifier for all Register, any number above 0x1D can used

/* NRF24 Init & Reset */
void NRF24_init(); 	//NRF24 Initialization
void NRF24_reset(uint8_t regrst);	//Reset a Register or all Register

/* NRF24 Command & Register Access */
void NRF24_send_cmd(uint8_t cmd);	//Send a Command
void NRF24_write_reg(uint8_t reg, uint8_t dat);	//Write Single byte to a Register
void NRF24_write_regN(uint8_t reg, uint8_t *Mdat, uint8_t Ndat); //Write Multiple byte to a Register
uint8_t NRF24_read_reg(uint8_t reg); //Read Single byte from a Register
void NRF24_read_regN(uint8_t reg, uint8_t *Mdat, uint8_t Ndat); //Read Multiple byte from a Register

/* NRF24 Transmit-Receive Function */
void NRF24_TXmode(uint8_t ch, uint8_t *addr); //Set as Transmitter
uint8_t NRF24_TX(uint8_t *Mdat, uint8_t datlen); //Transmit some byte of Data
void NRF24_RXmode(uint8_t ch, uint8_t npipe, uint8_t *addr, uint8_t ploadlen); //Set as Receiver
uint8_t NRF24_checkpipe(uint8_t npipe); //Check if Data is available in any Pipe
void NRF24_RX(uint8_t *Mdat, uint8_t datlen); //Receive some byte of Data

#endif /* INC_NRF24_H_ */
