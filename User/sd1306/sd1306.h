#ifndef __SD1306_I2C_H
#define	__SD1306_I2C_H

#include "stm32f10x.h"
#if defined(QST_USE_SW_I2C)
#include "qst_sw_i2c.h"
#else
#include "bsp_i2c.h"
#endif

#define FONT_SIZE8		8
#define FONT_SIZE16		16
#define XLevelL			0x02
#define XLevelH			0x10
#define SD1306_MAX_COL	128
#define SD1306_MAX_ROW	8
#define SD1306_WIDTH 	128
#define SD1306_HEIHT 	64

#define SD1306_SLAVE	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78

void sd1306_write_cmd(unsigned char I2C_Command);
void sd1306_write_data(unsigned char I2C_Data);
void sd1306_init(void);
void sd1306_display_on(void);
void sd1306_display_off(void);
void sd1306_fill_data(unsigned char fill_Data);
void sd1306_set_pos(unsigned char x, unsigned char y);
void sd1306_show_char(unsigned char x,unsigned char y,unsigned char chr,unsigned char Char_Size);
void sd1306_show_num(u8 x,u8 y,u32 num,u8 len,u8 size2);
void sd1306_show_str(u8 x,u8 y,u8 *chr,u8 Char_Size);

#endif
