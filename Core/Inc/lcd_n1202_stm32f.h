#ifndef __LCD_N1202_STM32F_H
#define __LCD_N1202_STM32F_H

#define LCDN1202_COL	96
#define LCDN1202_ROW	9	//8,5 (8 byte and 1 nibble)

#define LCDDAT 	GPIO_PIN_8
#define LCDCLK 	GPIO_PIN_9
#define LCDBL	GPIO_PIN_7
#define LCDP 	GPIOA
//#define LCDP_EN __HAL_RCC_GPIOA_CLK_ENABLE()

//LCD Hardware Access Functions
void lcdn1202_gpio_init(); //LCD Interface Initialization
void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat); //Send 9 bit cmd/data to DAT
void lcdn1202_clock1(); //Send clock once to CLK
//unsigned char font_read(unsigned int fontaddr); //Read Character Pattern from Font Array/Memory
void lcdn1202_blon();	//LCD-BL = '1'
void lcdn1202_bloff();	//LCD-BL = '0'

#endif
