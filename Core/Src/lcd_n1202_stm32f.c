#include "main.h"
#include "lcd_n1202_stm32f.h"

void lcdn1202_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCDP, LCDBL|LCDDAT|LCDCLK, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA7 PA8 PA9 */
	GPIO_InitStruct.Pin = LCDBL|LCDDAT|LCDCLK;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCDP, &GPIO_InitStruct);
}

void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat)
{
	unsigned char cdi;

	if(cdsign==0) HAL_GPIO_WritePin(LCDP, LCDDAT, GPIO_PIN_RESET); //LCDODR &= LCDDAT_MASKL;
	else HAL_GPIO_WritePin(LCDP, LCDDAT, GPIO_PIN_SET); //LCDODR |= LCDDAT_MASKH;
	lcdn1202_clock1();

	for(cdi=0;cdi<8;cdi++) //send 9 bit data
	   {
		if(comdat & 0x80) HAL_GPIO_WritePin(LCDP, LCDDAT, GPIO_PIN_SET); //LCDODR |= LCDDAT_MASKH; //Data = '1'
		else HAL_GPIO_WritePin(LCDP, LCDDAT, GPIO_PIN_RESET); //LCDODR &= LCDDAT_MASKL;		  //Data = '0'
		lcdn1202_clock1();
		comdat <<= 1;
	   }
	HAL_GPIO_WritePin(LCDP, LCDDAT, GPIO_PIN_RESET); //LCDODR &= LCDDAT_MASKL;
}

void lcdn1202_clock1()
{
	//LCDODR |= LCDCLK_MASKH;
	HAL_GPIO_WritePin(LCDP, LCDCLK, GPIO_PIN_SET);
	//HAL_Delay(1);
	__asm__("nop");
	//LCDODR &= LCDCLK_MASKL;
	HAL_GPIO_WritePin(LCDP, LCDCLK, GPIO_PIN_RESET);
}

//unsigned char font_read(unsigned int fontaddr)
//{
  	/*unsigned char *addr;

  	//Read Character Pattern from EEPROM
  	addr = (unsigned char*)EEPROM_BASE_ADDRESS + fontaddr;

  	return *addr;*/

	//unsigned char fontbyte;

	//Read Character Pattern from array
	//fontbyte = font_arr[fontaddr];

	//return fontbyte;
//}

void lcdn1202_blon()
{
	//LCDODR |= LCDBL_MASKH;
	HAL_GPIO_WritePin(LCDP, LCDBL, GPIO_PIN_SET);
}

void lcdn1202_bloff()
{
	//LCDODR &= LCDBL_MASKL;
	HAL_GPIO_WritePin(LCDP, LCDBL, GPIO_PIN_RESET);
}
