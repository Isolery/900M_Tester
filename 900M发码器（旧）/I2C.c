#include<iom16v.h>
#include<macros.h>
#include "I2C.h"

#define uchar  unsigned char
#define uint   unsigned int

void delay_ms(unsigned int x)
{
  unsigned int i,j;
  for(i=0;i<x;i++)
  for(j=0;j<100;j++); 
}

/*---------------I2C初始化--------------*/
void I2C_init(void)
{
 	SDA_OUT();
	SCL_OUT();
	SDA_H();
	SCL_H();
}
/*---------------I2C开始信号--------------*/
void I2C_Start(void)
{
 	I2C_init();
	delay_ms(1);
	SDA_L();
	delay_ms(1);
	SCL_L();
	delay_ms(1);
}
/*---------------I2C停止信号--------------*/
void I2C_Stop(void)
{
	SCL_L();
	SDA_OUT();
	SDA_L();
	delay_ms(1);
	SCL_H();
	delay_ms(1);
	SDA_H();
}

/*---------------I2C应答--------------*/
//I2C_Ack(0) >> 主机对从机产生应答信号
//I2C_Ack(1) >> 主机对从机产生非应答信号
void I2C_Ack(uchar ack)
{
 	SCL_L();
	SDA_OUT();
	if(ack==0)
	{
	 	SDA_L();
	}
	else
	{
	 	SDA_H();
	}
	delay_ms(1);
	SCL_H();
	delay_ms(1);
	SCL_L();
	delay_ms(1);
	SDA_H();
}
/*---------------I2C检测应答--------------*/
uchar I2C_Tack(void)
{
 	uint time=0;
	delay_ms(1);
    SDA_IN();
	while(PIND&0x10)
	{
	 	time++;
		if(time>250)
		{
		 	I2C_Stop();
			
			return 1;
		}
	}  
    delay_ms(1);
	SCL_H();
	delay_ms(1);
    SCL_L();
	delay_ms(1);
	SDA_OUT();
	SDA_H();
	delay_ms(1);
    return 0;//返回应答信号的状态，0表示应答，1表示非应答。
}

/*---------------I2C写操作--------------*/
void I2C_Writebyte(uchar dat)
{
 	uchar i;
	SDA_OUT();
	SDA_H();
	for(i=0;i<8;i++)
	{
	 	SCL_L();
		delay_ms(1);
		if(dat&0x80)
		{
		 	SDA_H();
			delay_ms(1);
		}
		else
		{
		 	SDA_L();
			delay_ms(1);
		}
		delay_ms(1);   //当SCL为低时SDA信号才能改变
		SCL_H();     //
		delay_ms(1);	 //
		dat<<=1;     // 
	}

	SCL_L();
	delay_ms(1);
	SDA_H();	
}

/*---------------I2C读操作--------------*/
uchar I2C_Readbyte(void)
{
 	 uchar i;
	 uchar rdat;
	 SCL_OUT();
	 SDA_OUT();
	 SCL_L();
	 delay_ms(1);
	 SDA_H();
	 delay_ms(1);
	 SDA_IN();
	 for(i=0;i<8;i++)
	 {
	  	SCL_L();
		delay_ms(1);
		SCL_H();
		delay_ms(1);
		rdat<<=1;
		if(PINC&0x02)
		{
		 	rdat|=0x01;		
		}
		delay_ms(1);
	 }
	 SCL_L();
	 delay_ms(1);
	 SDA_OUT();
	 return(rdat);
}

/*---------------向任意地址单字节写入4个字节--------------*/
uchar  Write_4Byte(uchar dadd,uchar wadd,uchar data[])        
{    
                                     
    I2C_Start();
    I2C_Writebyte(dadd);// 写入设备地址  
    if(I2C_Tack()==1) return (0);
    I2C_Writebyte(wadd);//写入要操作的单元地址
    if(I2C_Tack()==1) return (0);
    I2C_Writebyte(data[0]);//写入数据
    if(I2C_Tack()==1) return (0);
	I2C_Writebyte(data[1]);//写入数据
   if(I2C_Tack()==1) return (0);
	I2C_Writebyte(data[2]);//写入数据
   if(I2C_Tack()==1) return (0);
	I2C_Writebyte(data[3]);//写入数据
   if(I2C_Tack()==1) return (0);
    I2C_Stop();    
	return 1;                              											 
}

uchar  Write_2Byte1(uchar dadd,uchar wadd,uchar data[])        
{    
                                     
    I2C_Start();
    I2C_Writebyte(dadd);// 写入设备地址  
    if(I2C_Tack()==1) return (0);
    I2C_Writebyte(wadd);//写入要操作的单元地址
    if(I2C_Tack()==1) return (0);
    I2C_Writebyte(data[0]);//写入数据
    if(I2C_Tack()==1) return (0);
	I2C_Writebyte(data[1]);//写入数据
   if(I2C_Tack()==1) return (0);
    I2C_Stop();    
	return 1;                              											 
}

uchar  Write_2Byte2(uchar dadd,uchar wadd,uchar data[])  
{    
                                     
    I2C_Start();
    I2C_Writebyte(dadd);// 写入设备地址  
    if(I2C_Tack()==1) return (0);
    I2C_Writebyte(wadd);//写入要操作的单元地址
    if(I2C_Tack()==1) return (0);
    I2C_Writebyte(data[2]);//写入数据
    if(I2C_Tack()==1) return (0);
	I2C_Writebyte(data[3]);//写入数据
   if(I2C_Tack()==1) return (0);
    I2C_Stop();    
	return 1;                              											 
}




























































