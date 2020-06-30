#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "predefine.h"

/* TWI发送START信号，启动TWI传输 */
#define TWISTART()    (TWCR = (1<<TWSTA)|(1<<TWINT)|(1<<TWEN)|(1<<TWIE))    

/* 键码  */  // PB0 PB1 PB2 PB3 PB4 PB5 
#define WHITE    0x3E        //白灯按键码                    
#define BLUE       0x3D      //蓝灯按键码         
#define RED        0x3B       //红灯按键码
#define YGD1      0x37      //预告点1按键码
#define YGD2      0x2F      //预告点2按键码
#define YGD3      0x1F       //预告点3按键码

/* 灯码 */
#define CODE_W    0x51	//白灯码值
#define CODE_B     0x52	//蓝灯码值 
#define CODE_R     0x53	//红灯码值
#define CODE_Y     0x54	//预告点

#define DCI_ON    (PORTD |= (1<<7))
#define DCI_OFF   (PORTD &=~ (1<<7))

#define TRUE    1
#define FALSE   0

#define WDR()	  asm("wdr")

/* 设备地址 */
#define DEVICEADDR    0xA0

uint8_t WordAddr;    //内存地址
uint8_t keyValue;
uint8_t TransOk;
uint16_t tWDCount;

uint8_t writedata[2];          //写卡数据缓存

/*函数声明区*/
void TWI_Init();
void Port_Init(void);
void KeyScan(void);
void INT0_Init(void);
void WDT_Init(void);
void SystemInit(void);
void KeyHandle(void);

/**********************************************************************************************
* 函 数 名  ：main
* 功能说明：c程序入口
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
int main(void)
{
	SystemInit(); 

	while(1)
	{
		WDR();
		DCI_ON;
		_delay_ms(1000);
	}
	return 0;
}

/**********************************************************************************************
* 函 数 名  ：Port_Init
* 功能说明：端口初始化
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
void Port_Init(void)
{      
	   DDRA = 0x0f;PORTA = 0x0f;         //灯
	   DDRB = 0xc0;PORTB = 0x3f;        //按键
       DDRD = 0xff; 
	   DDRD &=~ (1<<2); PORTD |= (1<<2);   //INT0
}

/**********************************************************************************************
* 函 数 名  ：WDT_Init
* 功能说明：看门狗初始化，2s内没喂狗复位
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
void WDT_Init(void)
{
	  WDR();
	  WDTCR = ( (1<< WDTOE)|(1<< WDE) | (1 << WDP2) | (1 <<WDP1))|(1 << WDP0);   //设定周期为2S
}

/**********************************************************************************************
* 函 数 名  ：INT0_Init
* 功能说明：外部中断0初始化函数，低电平触发
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
void INT0_Init(void)
{
	GICR &= ~(1<<INT0);                     //关闭INT0中断使能
	MCUCR = 0x00;    //低电平触发
	GICR |= (1<<INT0);                        //开启INT0中断使能
}

/**********************************************************************************************
* 功能说明：外部中断0中断函数
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
ISR(INT0_vect)
{
	GICR &= ~(1<<INT0);	
	TransOk = FALSE;
	sei();    //置位全局中断标志位，让TWI中断可以发生		
	KeyScan();
	if(TransOk == FALSE)
	{
		PORTA = 0x00;
		_delay_ms(500);
	}
	PORTA = 0xFF;
	GICR |= (1<<INT0);    
}

/**********************************************************************************************
* 函 数 名  ：KeyScan
* 功能说明：按键扫描函数
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
void KeyScan(void)
{
    if((PINB&0x3f) != 0x3f)
	{
	     _delay_ms(10);
		 if((PINB&0x3f) != 0x3f)
		 {
		     keyValue = PINB;
			 PORTB = 0x3f;
			 KeyHandle();
		 }
	}          
}

/**********************************************************************************************
* 函 数 名  ：KeyHandle
* 功能说明：按键处理函数
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
void KeyHandle(void)
{
    switch(keyValue)
	{
	    case WHITE:          //白灯按键按下
		{
			PORTA &= ~(1<<3);           //点亮白灯		
			
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0x02;
			writedata[1] = CODE_W;
			WordAddr = 0x1E;
			TWISTART();    //启动TWI传输
			_delay_ms(50);
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0xF1;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//启动TWI传输

			PORTA |= (1<<3);	
			keyValue = 0x00;
			break;                    
		}
			
		case BLUE:          //蓝灯按键按下
		{	
			PORTA &= ~(1<<2);    //点亮蓝灯
				
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0x02;
			writedata[1] = CODE_B;
			WordAddr = 0x1E;
			TWISTART();    //启动TWI传输
			_delay_ms(50);
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0xF2;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//启动TWI传输
			
			PORTA |= (1<<2);
			keyValue = 0x00;
			break;                    
		}
			
		case RED:          //红灯按键按下
		{
			PORTA &=~ (1<<1);    //点亮红灯
				
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0x02;
			writedata[1] = CODE_R;
			WordAddr = 0x1E;
			TWISTART();    //启动TWI传输
			_delay_ms(50);
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0xF3;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//启动TWI传输
						
			PORTA |= (1<<1);
			keyValue = 0x00;
            break;                    
		}

		case YGD1:          //预告点1按键按下
		{
			PORTA &= ~(1<<0);    //点亮黄灯
				
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0x01;
			writedata[1] = CODE_Y;
			WordAddr = 0x1E;
			TWISTART();    //启动TWI传输
			_delay_ms(50);
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0xF1;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//启动TWI传输
			
			PORTA |= (1<<0);	
			keyValue = 0x00;
            break;                    
		}
			
		case YGD2:          //预告点2按键按下
		{
			PORTA &= ~(1<<0);	

			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0x01;
			writedata[1] = CODE_Y;
			WordAddr = 0x1E;
			TWISTART();    //启动TWI传输
			_delay_ms(50);
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0xF2;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//启动TWI传输
			
			PORTA |= (1<<0);	
			keyValue=0x00;
			break;                    
		}
			
		case YGD3:          //预告点3按键按下
		{
			PORTA &= ~(1<<0);	
				
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0x01;
			writedata[1] = CODE_Y;
			WordAddr = 0x1E;
			TWISTART();    //启动TWI传输
			_delay_ms(50);
			/* 将要发送的数据写入缓存数组 */
			writedata[0] = 0xF3;
			writedata[1] = 0x44;
			WordAddr = 0x20;
			TWISTART();//启动TWI传输
			
			PORTA |= (1<<0);	
			keyValue = 0x00;
            break;                    
		}
		default: break;
	}
}

/******************************************************************************************
* 函 数 名  ：TWI_Init
* 功能说明：TWI初始化 --> 波特率、TWCR
* 输入参数：void
* 返 回 值 ：void
*******************************************************************************************/
void TWI_Init()
{
	TWSR = 0;    //预分频设置为0
	TWBR = 15;    //Fscl = FOSC/(16+2·TWBR·4^TWPS)
	TWCR = (1<<TWEN)|(1<<TWIE);    //使能TWI接口、TWI中断; 其余位清零
}

/******************************************************************************************
* 功能说明：TWI中断函数，MT模式
* 输入参数：void
* 返 回 值 ：void
******************************************************************************************/
ISR(TWI_vect)
{
	uint8_t i;

	switch(TW_STATUS)
	{
		case TW_START:    //START已发送
		{
			/* 加载SLA+W */
			TWDR = DEVICEADDR;
			TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));     //写1清除TWINT标志位，启动TWI工作
			break;
		}
		case TW_REP_START:    //RESTART已发送
		{
			/* 加载SLA+W */
			TWDR = DEVICEADDR;
			TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));     //写1清除TWINT标志位，启动TWI工作
			break;
		}
		case TW_MT_SLA_ACK:    //SLA+W已发送且接收到ACK
		{
			/* 加载数据 */
			TWDR = WordAddr;    //ADDR
			TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));    //写1清除TWINT标志位，启动TWI工作
			break;
		}
		case TW_MT_SLA_NACK:    //SLA+W已发送且接收到NACK
		{
			/* RESTART */
			TWCR = ((1<<TWSTA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));
			break;
		}
		case TW_MT_DATA_ACK:    //数据已发送且接收到ACK
		{
			/* 继续发送数据 */
			for(i=0; i<2; i++)
			{
				TWDR = writedata[i];
				TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE));     //写1清除TWINT标志位，启动TWI工作
				while (!(TWCR & (1<<TWINT)));    //TWINT置位表示接收到从机的ACK，因此可以不用跳出中断继续执行for循环
			}
			TransOk = TRUE;
			TWCR = ((1<<TWSTO)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));    //发送STOP
			break;
		}
		case TW_MT_DATA_NACK:    //数据已发送且接收到NACK
		{
			/* RESTART */
			TWCR = ((1<<TWSTA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));
			break;
		}
		case TW_MT_ARB_LOST:    //SLA+W或数据的仲裁失败
		{
			/* 自动进入从机模式 */
			//TWCR = ((1<<TWSTA)|(1<<TWEN)|(1<<TWIE)|(1<<TWINT));
			break;
		}
		
		default: break;
	}
}

/************************************************************************************************
* 函 数 名 ：SystemInit
* 功能说明：系统初始化函数 \ 端口初始化，看门狗初始化，外部中断初始化，TWI初始化
* 输入参数：void
* 返 回 值 ：void
************************************************************************************************/
void SystemInit(void)
{
	WDR();
	Port_Init();
	WDT_Init();
	INT0_Init();
	TWI_Init();
	sei();
}