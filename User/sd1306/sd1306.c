/**
  ******************************************************************************
  * @file    OLED_I2C.c
  * @author  fire
  * @version V1.0
  * @date    2014-xx-xx
  * @brief   128*64点阵的OLED显示屏驱动文件，仅适用于SD1306驱动IIC通信方式显示屏
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 ISO STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
	
	* Function List:
	*	1. void I2C_Configuration(void) -- 配置CPU的硬件I2C
	* 2. void sd1306_write_byte(uint8_t addr,uint8_t data) -- 向寄存器地址写一个byte的数据
	* 3. void sd1306_write_cmd(unsigned char I2C_Command) -- 写命令
	* 4. void sd1306_write_data(unsigned char I2C_Data) -- 写数据
	* 5. void sd1306_init(void) -- OLED屏初始化
	* 6. void sd1306_set_pos(unsigned char x, unsigned char y) -- 设置起始点坐标
	* 7. void sd1306_fill(unsigned char fill_Data) -- 全屏填充
	* 8. void sd1306_cls(void) -- 清屏
	* 9. void sd1306_on(void) -- 唤醒
	* 10. void sd1306_off(void) -- 睡眠
	* 11. void sd1306_show_str(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) -- 显示字符串(字体大小有6*8和8*16两种)
	* 12. void sd1306_show_cn(unsigned char x, unsigned char y, unsigned char N) -- 显示中文(中文需要先取模，然后放到codetab.h中)
	* 13. void sd1306_drae_bmp(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]) -- BMP图片
	*
  *
  ******************************************************************************
  */ 




#include "sd1306.h"
//#include "codetab.h"
#include "oledfont.h"
//#include "bmp.h"

#if 0
void sd1306_write_cmd(unsigned char I2C_Command)//写命令
{
	qst_sw_writereg(SD1306_SLAVE, 0x00, I2C_Command);
}


void sd1306_write_data(unsigned char I2C_Data)//写数据
{
	//sd1306_write_byte(0x40, I2C_Data);	
	qst_sw_writereg(SD1306_SLAVE, 0x40, I2C_Data);
}
#else
#define sd1306_write_cmd(I2C_Command)			qst_sw_writereg(SD1306_SLAVE, 0x00, I2C_Command)
#define sd1306_write_data(I2C_Command)			qst_sw_writereg(SD1306_SLAVE, 0x40, I2C_Command)
#endif

 
 void sd1306_delay(unsigned int Del_1ms)
 {
	 unsigned char j;
	 while(Del_1ms--)
	 {	 
		 for(j=0;j<123;j++);
	 }
 }

void OLED_WR_Byte(unsigned dat, unsigned cmd)
{
	if(cmd)
   		sd1306_write_data(dat);
	else 
   		sd1306_write_cmd(dat);
}

/********************************************
// fill_Picture
********************************************/
void sd1306_fill_data(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0; m<SD1306_MAX_ROW; m++)
	{
		sd1306_write_cmd(0xb0+m);		//page0-page1
		sd1306_write_cmd(0x00);		//low column start address
		sd1306_write_cmd(0x10);		//high column start address
		for(n=0; n<SD1306_MAX_COL; n++)
		{
			sd1306_write_data(fill_Data);
		}
	}
}


//坐标设置
void sd1306_set_pos(unsigned char x, unsigned char y) 
{ 	
	sd1306_write_cmd(0xb0+y);
	sd1306_write_cmd(((x&0xf0)>>4)|0x10);
	sd1306_write_cmd((x&0x0f)); 
}

//开启OLED显示    
void sd1306_display_on(void)
{
	sd1306_write_cmd(0X8D);  //SET DCDC命令
	sd1306_write_cmd(0X14);  //DCDC ON
	sd1306_write_cmd(0XAF);  //DISPLAY ON
}
//关闭OLED显示     
void sd1306_display_off(void)
{
	sd1306_write_cmd(0X8D);  //SET DCDC命令
	sd1306_write_cmd(0X10);  //DCDC OFF
	sd1306_write_cmd(0XAE);  //DISPLAY OFF
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void sd1306_show_char(unsigned char x,unsigned char y,unsigned char chr,unsigned char Char_Size)
{      	
	unsigned char c=0,i=0;

	c=chr-' ';//得到偏移后的值	
	if(x>SD1306_MAX_COL-1)
	{
		x=0;
		y=y+2;
	}
	if(Char_Size ==16)
	{
		sd1306_set_pos(x,y);	
		for(i=0;i<8;i++)
		{
			sd1306_write_data(F8X16[c*16+i]);
		}
		sd1306_set_pos(x,y+1);
		for(i=0;i<8;i++)
		{
			sd1306_write_data(F8X16[c*16+i+8]);
		}
	}
	else 
	{	
		sd1306_set_pos(x,y);
		for(i=0;i<6;i++)
		{
			sd1306_write_data(F6x8[c][i]);
		}
	}
}

//m^n函数
u32 sd1306_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void sd1306_show_num(u8 x,u8 y,u32 num,u8 len,u8 size2)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/sd1306_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				sd1306_show_char(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	sd1306_show_char(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 

//显示一个字符号串
void sd1306_show_str(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		
		sd1306_show_char(x,y,chr[j],Char_Size);
		x+=8;
		if(x>120)
		{
			x=0;
			y+=2;
		}
		j++;
	}
}

//显示汉字
void sd1306_show_ch(u8 x,u8 y,u8 no)
{      			    
	u8 t,adder=0;
	sd1306_set_pos(x,y);	
	for(t=0;t<16;t++)
	{
		sd1306_write_data(Hzk[2*no][t]);
		adder+=1;
	}
	sd1306_set_pos(x,y+1);	
    for(t=0;t<16;t++)
	{	
		sd1306_write_data(Hzk[2*no+1][t]);
		adder+=1;
	}					
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void sd1306_draw_bmp(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
	unsigned int j=0;
	unsigned char x,y;

	if(y1%8==0) 
		y=y1/8;
	else
		y=y1/8+1;

	for(y=y0;y<y1;y++)
	{
		sd1306_set_pos(x0,y);
		for(x=x0;x<x1;x++)
	    {      
	    	sd1306_write_data(BMP[j++]);
	    }
	}
} 

//初始化SSD1306					    
void sd1306_init(void)
{ 	
	sd1306_delay(200);
	sd1306_write_cmd(0xAE);//关闭显示
	
	sd1306_write_cmd(0x40);//---set low column address
	sd1306_write_cmd(0xB0);//---set high column address

	sd1306_write_cmd(0xC8);//-not offset

	sd1306_write_cmd(0x81);//设置对比度
	sd1306_write_cmd(0xff);

	sd1306_write_cmd(0xa1);//段重定向设置

	sd1306_write_cmd(0xa6);//
	
	sd1306_write_cmd(0xa8);//设置驱动路数
	sd1306_write_cmd(0x1f);
	
	sd1306_write_cmd(0xd3);
	sd1306_write_cmd(0x00);
	
	sd1306_write_cmd(0xd5);
	sd1306_write_cmd(0xf0);
	
	sd1306_write_cmd(0xd9);
	sd1306_write_cmd(0x22);
	
	sd1306_write_cmd(0xda);
	sd1306_write_cmd(0x02);
	
	sd1306_write_cmd(0xdb);
	sd1306_write_cmd(0x49);
	
	sd1306_write_cmd(0x8d);
	sd1306_write_cmd(0x14);
	
	sd1306_write_cmd(0xaf);
	sd1306_fill_data(0);
}  

