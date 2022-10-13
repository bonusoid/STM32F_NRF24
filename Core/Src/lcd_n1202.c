#include "main.h"
#include "lcd_n1202.h"
#include "lcd_n1202_stm32f.h"
#include "font.h"

void lcdn1202_init()
{
	lcdn1202_gpio_init();

	//Hard Reset -> By HW using R-C

	HAL_Delay(10);

	lcdn1202_sendcom(0xE2);	//Soft Reset
	HAL_Delay(1);
	lcdn1202_sendcom(0xA4); //Normal Display Mode
	lcdn1202_sendcom(0x2F);	//Power Control = Max (Booster On, VReg On, VFol On)

	lcdn1202_sendcom(0xA0); //Segment Driver Direction = Normal (lines start at left)
	lcdn1202_sendcom(0xC0); //Common Driver Direction = Normal
	lcdn1202_sendcom(0x80|16); //Set Contrast to Default

	lcdn1202_sendcom(0xAF);	//Display On

	LCD_BL_OFF(); //Backlight Off
	LCD_clear();  //Clear pixel memory
	LCD_BL_ON();  //Backlight On
}

void lcdn1202_sendcom(unsigned char ssd1306com)
{
	lcdn1202_9bsend(0,ssd1306com);
}

void lcdn1202_senddat(unsigned char ssd1306dat)
{
	lcdn1202_9bsend(1,ssd1306dat);
}

void lcdn1202_setpos(unsigned char row, unsigned char col)
{
	lcdn1202_sendcom(0xB0|(row&0x0F)); //Set page of row
	lcdn1202_sendcom(0x00|(col&0x0F)); //Set lower nibble of Column
	lcdn1202_sendcom(0x10|((col>>4)&0x0F)); //Set upper nibble of Column
}

void lcdn1202_clear()
{
	unsigned char col,row;
	lcdn1202_setpos(0,0);
  	for(row=0;row<LCDN1202_ROW;row++)	//scan rows (pages)
  	   {
      		for(col=0;col<LCDN1202_COL;col++)	//scan columns
      		   {
        		lcdn1202_senddat(0);	//send 0 to the all pixel
      		   }
  	   }
}

void LCD_setpos(unsigned char row, unsigned char col)
{
	lcdn1202_setpos(row,col);
}

void LCD_drawbyte(unsigned char dbyte)
{
	lcdn1202_senddat(dbyte);
}

void LCD_drawchar(unsigned char chr, unsigned char chrrow, unsigned char chrcol)
{
	unsigned char ci,fchar;
	unsigned int chridx;

	lcdn1202_setpos(chrrow,chrcol);

	if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation
	  {
	    lcdn1202_senddat(0x00);
            chridx=(chr-32)*5;
            for(ci=0;ci<5;ci++)
		{
		   //fchar = font_read(chridx+ci); //get Character Pattern
		   fchar = font_arr[chridx+ci]; //get Character Pattern from Font Array
		   lcdn1202_senddat(fchar);
		}
          }
 	else if((chr>127)&&(chr<148))	//Frame & Arrow
	  {
	    chridx=(chr-128)*8;
            for(ci=0;ci<8;ci++)
		{
 		   //fchar = font_read(chridx+480+ci); //get Character Pattern
 		   fchar = font_arr[chridx+480+ci]; //get Character Pattern from Font Array
		   lcdn1202_senddat(fchar);
		}
	  }
	else{}
}

void LCD_drawtext(char *text, unsigned char txtrow, unsigned char txtcol)
{
	unsigned int stridx = 0;

	while(text[stridx] != 0) //scan characters in string
	  {
		LCD_drawchar(text[stridx],txtrow,txtcol+(8*stridx)); //print each character
		stridx++;
	  }
}

void LCD_drawint(unsigned int num, unsigned char numrow, unsigned char numcol)
{
	char ibuff[11]; //MAX : 5 DIGIT : 65535

	unsigned char ndigit=0,nd;
	unsigned int numb; //must unsigned, so max. number can be 65535
			   //if set to signed, max. number only 55536

	numb = num;
	while(numb!=0)
	  {
	  	ndigit++;
		numb /= 10; //count decimal digit
	  }
	for(nd=0;nd<ndigit;nd++)
	  {
		numb = num%10;
		num = num/10;
		ibuff[ndigit-(nd+1)] = numb + '0'; //start from last_index-1
	  }
	ibuff[ndigit] = '\0'; //last character is null

	LCD_drawtext(ibuff,numrow,numcol);
}

void LCD_clear()
{
	lcdn1202_sendcom(0xAE);  //Set Display Off
	lcdn1202_clear(); //Clear Display
	lcdn1202_sendcom(0xAF); //Set Display On
}

void LCD_clearblock(unsigned char row, unsigned char col_start, unsigned char col_fin)
{
	unsigned char col;

	lcdn1202_setpos(row,col_start);
	for(col=col_start;col<=col_fin;col++) //scan specific columns
	   {
		lcdn1202_senddat(0);	//send 0 to the all pixel
	   }
}

void LCD_normal()
{
	lcdn1202_sendcom(0xA6);	//Black Pixel in White Background
}

void LCD_reverse()
{
	lcdn1202_sendcom(0xA7);	//White Pixel in Black Background
}

void LCD_BL_ON()
{
	lcdn1202_blon();
}

void LCD_BL_OFF()
{
	lcdn1202_bloff();
}
