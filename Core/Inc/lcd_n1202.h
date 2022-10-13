#ifndef __LCD_N1202_H
#define __LCD_N1202_H

#define SYM_DEGREE       127

#define FRAME_TOP_LEFT   128
#define FRAME_TOP	     129
#define FRAME_TOP_RIGHT  130

#define FRAME_MID_LEFT   131
#define FRAME_CENTER	 132
#define FRAME_MID_RIGHT  133

#define FRAME_BOT_LEFT   134
#define FRAME_BOT	     135
#define FRAME_BOT_RIGHT  136

#define FRAME_LINE_HOR   137
#define FRAME_LINE_VER   138

#define ARROW_UP         139
#define ARROW_DOWN       140
#define ARROW_LEFT       141
#define ARROW_RIGHT      142
#define ARROW_UP_LEFT    143
#define ARROW_UP_RIGHT   144
#define ARROW_DOWN_LEFT  145
#define ARROW_DOWN_RIGHT 146
#define ARROW_POINT	 	 147

//LCD Operational Functions
void lcdn1202_init();	//LCD Initialization
void lcdn1202_sendcom(unsigned char ssd1306com);	//Send Command
void lcdn1202_senddat(unsigned char ssd1306dat);	//Send Data
void lcdn1202_setpos(unsigned char row, unsigned char col);	//Set Coordinate
void lcdn1202_clear();	//Clear all pixel

void LCD_setpos(unsigned char row, unsigned char col);	//Set Coordinate (for OLED_drawbyte)
void LCD_drawbyte(unsigned char dbyte);	//Draw One Byte
void LCD_drawchar(unsigned char chr, unsigned char chrrow, unsigned char chrcol);	//Draw One Character
void LCD_drawtext(char *text, unsigned char txtrow, unsigned char txtcol);	//Draw Text
void LCD_drawint(unsigned int num, unsigned char numrow, unsigned char numcol);	//Draw Number (Integer), MAX=65535
void LCD_clear();	//Reset/Clear Screen
void LCD_clearblock(unsigned char row, unsigned char col_start, unsigned char col_fin);	//Reset/Clear Area
void LCD_normal();	//Normal Color
void LCD_reverse();	//Reverse Color

void LCD_BL_ON();	//Backlight On
void LCD_BL_OFF();	//Backlight Off

#endif
